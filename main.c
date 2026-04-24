/******************************************************************************
 * @file    main.c
 * @brief   CANOpNode_Sys 主程序 - 重力卸载控制系统
 * @author  System Architect
 * @date    2026-04-23
 * @version 2.0.0
 * 
 * @description
 * 重力卸载控制系统主程序
 * 1. 初始化所有硬件设备
 * 2. 执行系统预检测
 * 3. 等待用户确认
 * 4. 启动重力卸载算法
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>

#include "config/system_config.h"
#include "utils/logger.h"
#include "drivers/sensor_manager.h"
#include "drivers/power_driver.h"
#include "drivers/motor_driver.h"
#include "algorithms/gravity_unload.h"
#include "algorithms/system_check.h"

/******************************************************************************
 * 全局变量
 ******************************************************************************/
static volatile int g_running = 1;
static volatile int g_algorithm_enabled = 0;

static SensorManager_t g_sensor_mgr;
static PowerDriver_t g_power;
static MotorDriver_t g_motor;
static GravityUnloadController_t g_gravity_ctrl;

static int g_motor_enabled = 1;  /* 电机使能标志 */

/******************************************************************************
 * 外部接口实现（供算法模块调用）
 ******************************************************************************/

uint32_t get_timestamp_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

int get_sensor_data(SensorDataRaw_t *data) {
    if (data == NULL) return -1;
    
    SensorData_t encoder_data, pressure_data;
    
    /* 获取编码器数据 */
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data) != ERR_OK) {
        return -1;
    }
    
    /* 获取压力传感器数据 */
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure_data) != ERR_OK) {
        return -1;
    }
    
    /* 填充原始数据 */
    data->pressure_kg = pressure_data.data.pressure.pressure_kg;
    data->encoder_position_m = encoder_data.data.encoder.rope_length_mm / 1000.0f; /* mm转m */
    data->timestamp_ms = get_timestamp_ms();
    data->data_valid = (encoder_data.data_valid && pressure_data.data_valid);
    
    return 0;
}

int set_motor_velocity(float velocity) {
    if (!g_motor_enabled || !g_motor.initialized) return -1;
    return (motor_set_velocity(&g_motor, (int32_t)velocity) == ERR_OK) ? 0 : -1;
}

int get_motor_actual_velocity(float *velocity) {
    if (!g_motor_enabled || !g_motor.initialized || velocity == NULL) return -1;
    
    int32_t vel;
    if (motor_get_velocity(&g_motor, &vel) == ERR_OK) {
        *velocity = (float)vel;
        return 0;
    }
    return -1;
}

int set_clutch_current(float current_mA) {
    /* 控制电源板电流输出 */
    uint16_t current_u16 = (uint16_t)(current_mA + 0.5f);
    
    /* 限制范围 */
    if (current_u16 > SAFETY_CLUTCH_CURRENT_MAX_MA) {
        current_u16 = SAFETY_CLUTCH_CURRENT_MAX_MA;
    }
    
    return (power_set_current(&g_power, current_u16) == ERR_OK) ? 0 : -1;
}

/******************************************************************************
 * 信号处理
 ******************************************************************************/
void signal_handler(int sig) {
    (void)sig;
    printf("\n[INFO] Signal received, shutting down...\n");
    g_running = 0;
}

/******************************************************************************
 * 系统预检测函数
 ******************************************************************************/
static int check_sensors_impl(void) {
    SensorData_t encoder, pressure;
    
    /* 检查编码器 */
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder) != ERR_OK) {
        printf("[CHECK FAIL] Encoder data not available\n");
        return -1;
    }
    if (!encoder.data_valid) {
        printf("[CHECK FAIL] Encoder data invalid\n");
        return -1;
    }
    
    /* 检查压力传感器 */
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure) != ERR_OK) {
        printf("[CHECK FAIL] Pressure sensor data not available\n");
        return -1;
    }
    if (!pressure.data_valid) {
        printf("[CHECK FAIL] Pressure sensor data invalid\n");
        return -1;
    }
    
    printf("[CHECK PASS] Sensors OK (Pressure: %.3f kg, Encoder: %.3f m)\n",
           pressure.data.pressure.pressure_kg,
           encoder.data.encoder.rope_length_mm / 1000.0f);
    return 0;
}

static int check_motor_impl(void) {
    if (!g_motor_enabled) {
        printf("[CHECK SKIP] Motor disabled\n");
        return 0;
    }
    
    if (!g_motor.initialized) {
        printf("[CHECK FAIL] Motor not initialized\n");
        return -1;
    }
    
    /* 尝试读取电机状态 */
    if (motor_update_state(&g_motor) != ERR_OK) {
        printf("[CHECK FAIL] Cannot communicate with motor\n");
        return -1;
    }
    
    printf("[CHECK PASS] Motor OK (NodeID=%d)\n", g_motor.node_id);
    return 0;
}

static int check_power_impl(void) {
    uint16_t current, voltage;
    
    if (power_get_status(&g_power, &current, &voltage) != ERR_OK) {
        printf("[CHECK FAIL] Cannot communicate with power board\n");
        return -1;
    }
    
    printf("[CHECK PASS] Power board OK (%.2f V, %.3f A)\n",
           voltage / 1000.0f, current / 1000.0f);
    return 0;
}

static int check_safety_impl(void) {
    SensorData_t pressure;
    
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure) != ERR_OK) {
        return -1;
    }
    
    /* 检查压力是否在安全范围 */
    if (pressure.data.pressure.pressure_kg < SAFETY_PRESSURE_MIN_KG ||
        pressure.data.pressure.pressure_kg > SAFETY_PRESSURE_MAX_KG) {
        printf("[CHECK FAIL] Pressure out of safety range: %.3f kg\n",
               pressure.data.pressure.pressure_kg);
        return -1;
    }
    
    printf("[CHECK PASS] Safety check OK\n");
    return 0;
}

/******************************************************************************
 * 用户确认函数
 ******************************************************************************/
static int request_user_confirmation_impl(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║                                                              ║\n");
    printf("║         重力卸载控制系统 - 启动确认                          ║\n");
    printf("║                                                              ║\n");
    printf("╠══════════════════════════════════════════════════════════════╣\n");
    printf("║                                                              ║\n");
    printf("║  ⚠️  系统即将启动重力卸载算法                                ║\n");
    printf("║                                                              ║\n");
    printf("║  请确认：                                                    ║\n");
    printf("║  1. 所有设备已正确连接                                       ║\n");
    printf("║  2. 重物已正确悬挂                                           ║\n");
    printf("║  3. 人员已远离运动区域                                       ║\n");
    printf("║  4. 紧急情况可以按 Ctrl+C 停止                               ║\n");
    printf("║                                                              ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    /* 等待用户输入 */
    printf("请输入 'yes' 启动算法，或其他键取消: ");
    fflush(stdout);
    
    char input[16];
    if (fgets(input, sizeof(input), stdin) == NULL) {
        return 0;
    }
    
    /* 去掉换行符 */
    input[strcspn(input, "\n")] = 0;
    
    if (strcmp(input, "yes") == 0 || strcmp(input, "YES") == 0) {
        printf("\n[INFO] 用户确认，启动算法...\n");
        return 1;
    }
    
    printf("\n[INFO] 用户取消，算法未启动\n");
    return 0;
}

/******************************************************************************
 * 监控线程 - 1Hz打印状态
 ******************************************************************************/
void* monitor_thread(void* arg) {
    (void)arg;
    
    printf("[MONITOR] Monitor thread started (1Hz)\n");
    
    while (g_running) {
        /* 打印重力卸载控制器状态 */
        if (g_algorithm_enabled) {
            gravity_unload_print_status(&g_gravity_ctrl);
        }
        
        sleep(1);
    }
    
    printf("[MONITOR] Monitor thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 主函数
 ******************************************************************************/
int main(int argc, char *argv[]) {
    /* 解析命令行参数 */
    if (argc > 1 && (strcmp(argv[1], "--no-motor") == 0 || strcmp(argv[1], "-n") == 0)) {
        g_motor_enabled = 0;
        printf("Motor control disabled\n");
    } else {
        printf("Motor control enabled (use -n or --no-motor to disable)\n");
    }
    fflush(stdout);
    
    /* 注册信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* 初始化日志 */
    logger_init(NULL, NULL, LOG_LEVEL_INFO, 1);
    
    printf("========================================\n");
    printf("CANOpNode_Sys v2.0.0 (Gravity Unload)\n");
    printf("========================================\n\n");
    fflush(stdout);
    
    /* ========== 阶段1: 初始化硬件 ========== */
    printf("[INIT] Phase 1: Initializing hardware...\n");
    
    /* 1. 初始化统一传感器管理器 */
    printf("  -> Sensor manager... ");
    fflush(stdout);
    if (sensor_mgr_init(&g_sensor_mgr, ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE) != ERR_OK) {
        printf("FAILED\n");
        return 1;
    }
    printf("OK\n");
    
    sensor_mgr_set_encoder_rope_params(&g_sensor_mgr, 100.0f, 4096);
    
    /* 执行校准 */
    printf("  -> Calibrating sensors... ");
    fflush(stdout);
    sensor_mgr_encoder_zero_calibration(&g_sensor_mgr);
    sensor_mgr_pressure_tare(&g_sensor_mgr);
    printf("OK\n");
    
    /* 2. 初始化电源板 */
    printf("  -> Power driver... ");
    fflush(stdout);
    if (power_init(&g_power, POWER_UART_DEVICE, POWER_UART_BAUDRATE) != ERR_OK) {
        printf("FAILED\n");
        sensor_mgr_deinit(&g_sensor_mgr);
        return 1;
    }
    printf("OK\n");
    
    /* 3. 初始化电机 */
    if (g_motor_enabled) {
        printf("  -> Motor driver... ");
        fflush(stdout);
        if (motor_init(&g_motor, MOTOR_NODE_ID, MOTOR_CAN_INTERFACE) != ERR_OK) {
            printf("WARNING (non-critical)\n");
            g_motor_enabled = 0;
        } else {
            printf("OK\n");
        }
    }
    
    /* 4. 启动传感器管理线程 */
    printf("  -> Starting sensor manager thread... ");
    fflush(stdout);
    if (sensor_mgr_start(&g_sensor_mgr) != ERR_OK) {
        printf("FAILED\n");
        power_deinit(&g_power);
        sensor_mgr_deinit(&g_sensor_mgr);
        return 1;
    }
    printf("OK\n");
    
    /* 等待传感器稳定 */
    printf("  -> Waiting for sensors to stabilize... ");
    fflush(stdout);
    sleep(2);
    printf("OK\n\n");
    
    /* ========== 阶段2: 系统预检测 ========== */
    printf("[INIT] Phase 2: System health check...\n");
    
    int check_pass = 1;
    
    if (check_sensors_impl() != 0) check_pass = 0;
    if (check_motor_impl() != 0) check_pass = 0;
    if (check_power_impl() != 0) check_pass = 0;
    if (check_safety_impl() != 0) check_pass = 0;
    
    if (!check_pass) {
        printf("\n[ERROR] System check FAILED! Please check hardware connections.\n");
        sensor_mgr_stop(&g_sensor_mgr);
        power_deinit(&g_power);
        sensor_mgr_deinit(&g_sensor_mgr);
        return 1;
    }
    
    printf("\n[INIT] All system checks PASSED!\n\n");
    
    /* ========== 阶段3: 用户确认 ========== */
    printf("[INIT] Phase 3: User confirmation\n");
    
    if (!request_user_confirmation_impl()) {
        printf("\n[INFO] Startup cancelled by user\n");
        sensor_mgr_stop(&g_sensor_mgr);
        power_deinit(&g_power);
        sensor_mgr_deinit(&g_sensor_mgr);
        return 0;
    }
    
    /* ========== 阶段4: 初始化算法 ========== */
    printf("\n[INIT] Phase 4: Initializing gravity unload algorithm...\n");
    
    if (gravity_unload_init(&g_gravity_ctrl) != 0) {
        printf("[ERROR] Failed to initialize algorithm\n");
        sensor_mgr_stop(&g_sensor_mgr);
        power_deinit(&g_power);
        sensor_mgr_deinit(&g_sensor_mgr);
        return 1;
    }
    
    /* ========== 阶段5: 启动算法 ========== */
    printf("[INIT] Phase 5: Starting algorithm...\n");
    
    if (gravity_unload_start(&g_gravity_ctrl) != 0) {
        printf("[ERROR] Failed to start algorithm\n");
        gravity_unload_deinit(&g_gravity_ctrl);
        sensor_mgr_stop(&g_sensor_mgr);
        power_deinit(&g_power);
        sensor_mgr_deinit(&g_sensor_mgr);
        return 1;
    }
    
    g_algorithm_enabled = 1;
    
    /* 创建监控线程 */
    pthread_t monitor_tid;
    pthread_create(&monitor_tid, NULL, monitor_thread, NULL);
    
    printf("\n========================================\n");
    printf("System RUNNING - Gravity unload active\n");
    printf("Press Ctrl+C to stop\n");
    printf("========================================\n\n");
    fflush(stdout);
    
    /* ========== 主循环 ========== */
    while (g_running) {
        /* 检查算法状态 */
        AlgoStatus_t status;
        gravity_unload_get_status(&g_gravity_ctrl, &status);
        
        if (status.state == ALGO_STATE_EMERGENCY_STOP) {
            printf("\n[EMERGENCY] Algorithm stopped due to safety violation!\n");
            break;
        }
        
        if (status.error != ALGO_ERR_NONE && status.error != ALGO_ERR_INVALID_PARAM) {
            printf("\n[WARNING] Algorithm error: %d\n", status.error);
        }
        
        sleep(1);
    }
    
    /* ========== 清理 ========== */
    printf("\n[SHUTDOWN] Stopping system...\n");
    
    /* 停止算法 */
    printf("  -> Stopping algorithm... ");
    fflush(stdout);
    gravity_unload_stop(&g_gravity_ctrl);
    gravity_unload_deinit(&g_gravity_ctrl);
    printf("OK\n");
    
    /* 停止传感器管理器 */
    printf("  -> Stopping sensor manager... ");
    fflush(stdout);
    sensor_mgr_stop(&g_sensor_mgr);
    printf("OK\n");
    
    /* 等待监控线程结束 */
    pthread_join(monitor_tid, NULL);
    
    /* 打印最终状态 */
    printf("\n");
    sensor_mgr_print_status(&g_sensor_mgr);
    
    /* 反初始化设备 */
    power_deinit(&g_power);
    sensor_mgr_deinit(&g_sensor_mgr);
    
    printf("\n[System shutdown completed]\n");
    logger_deinit(NULL);
    
    return 0;
}
