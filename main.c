/******************************************************************************
 * @file    main.c
 * @brief   CANOpNode_Sys 主程序 - 重力卸载控制系统 v3.0
 * @author  System Architect
 * @date    2026-05-11
 * @version 3.0.0
 * 
 * @description
 * 重力卸载控制系统主程序
 * - 共享内存通信：20Hz实时数据输出
 * - 算法独立监控：无论算法是否运行都可监控和控制
 * - 双向通信：数据上传 + 命令接收
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
#include "include/shared_memory.h"
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
static volatile int g_shm_initialized = 0;
static volatile uint32_t g_last_cmd_id = 0;

static SensorManager_t g_sensor_mgr;
static PowerDriver_t g_power;
static MotorDriver_t g_motor;
static GravityUnloadController_t g_gravity_ctrl;
static ShmManager_t g_shm_mgr;

static int g_motor_enabled = 1;  /* 电机使能标志 */
static int g_algorithm_mode = 0; /* 0=手动模式, 1=算法模式 */

/* 线程ID */
static pthread_t g_data_thread_tid;
static pthread_t g_command_thread_tid;
static pthread_t g_monitor_thread_tid;
static pthread_t g_collection_thread_tid;

/* 共享状态缓冲区 - 用于线程间数据共享 */
typedef struct {
    pthread_mutex_t mutex;
    /* 传感器数据 */
    float pressure_kg;
    float rope_length_m;
    uint32_t encoder_value;
    float encoder_angle_deg;
    /* 电源数据 */
    float current_a;
    float voltage_v;
    /* 电机数据 */
    float motor_speed_rpm;
    float motor_position_m;
    int32_t motor_status;
} SharedStateBuffer_t;

static SharedStateBuffer_t g_shared_state;

/* 初始化共享状态缓冲区 */
static void shared_state_init(void) {
    pthread_mutex_init(&g_shared_state.mutex, NULL);
    memset((void*)&g_shared_state + sizeof(pthread_mutex_t), 0, 
           sizeof(SharedStateBuffer_t) - sizeof(pthread_mutex_t));
}

/* 更新传感器数据到共享缓冲区 */
static void update_sensor_to_buffer(SensorData_t *encoder, SensorData_t *pressure) {
    pthread_mutex_lock(&g_shared_state.mutex);
    if (encoder && encoder->data_valid) {
        g_shared_state.encoder_value = encoder->data.encoder.multi_turn_value;
        g_shared_state.encoder_angle_deg = encoder->data.encoder.angle_deg;
        g_shared_state.rope_length_m = encoder->data.encoder.rope_length_mm * 0.001f; // 转换为米
    }
    if (pressure && pressure->data_valid) {
        g_shared_state.pressure_kg = pressure->data.pressure.pressure_kg;
    }
    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 更新电源数据到共享缓冲区 */
static void update_power_to_buffer(PowerDriver_t *power) {
    pthread_mutex_lock(&g_shared_state.mutex);
    if (power && power->state == POWER_STATE_ON) {
        g_shared_state.current_a = (float)power->actual_current / 1000.0f; // mA -> A
        g_shared_state.voltage_v = (float)power->actual_voltage / 1000.0f; // mV -> V
    }
    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 更新电机数据到共享缓冲区 */
static void update_motor_to_buffer(void) {
    pthread_mutex_lock(&g_shared_state.mutex);
    // 从电机驱动获取最新数据
    g_shared_state.motor_speed_rpm = (float)motor_get_velocity_rpm(&g_motor);
    g_shared_state.motor_position_m = (float)motor_get_position_m(&g_motor);
    g_shared_state.motor_status = g_motor.state;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

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
    data->encoder_position_m = encoder_data.data.encoder.rope_length_mm / 1000.0f;
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
    uint16_t current_u16 = (uint16_t)(current_mA + 0.5f);
    
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
    
    /* 确保保存编码器数据 */
    sensor_mgr_deinit(&g_sensor_mgr);
    power_deinit(&g_power);
}

/******************************************************************************
 * 系统预检测函数
 ******************************************************************************/
static int check_sensors_impl(void) {
    SensorData_t encoder, pressure;
    
    printf("[CHECK] Checking sensors...\n");
    fflush(stdout);
    
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder) != ERR_OK) {
        printf("[CHECK FAIL] Encoder data not available\n");
        return -1;
    }
    if (!encoder.data_valid) {
        printf("[CHECK FAIL] Encoder data invalid\n");
        return -1;
    }
    
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
    
    printf("[CHECK] Checking motor communication... ");
    fflush(stdout);
    
    int retries = 3;
    int motor_ok = 0;
    while (retries-- > 0) {
        if (motor_update_state(&g_motor) == ERR_OK) {
            motor_ok = 1;
            break;
        }
        usleep(100000);
    }
    
    if (!motor_ok) {
        printf("FAILED\n");
        printf("[CHECK FAIL] Cannot communicate with motor after retries\n");
        return -1;
    }
    
    printf("OK\n");
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
 * 共享内存数据线程 - 20Hz输出
 ******************************************************************************/
static void* data_output_thread(void* arg) {
    (void)arg;
    
    printf("[SHM] Data output thread started (20Hz)\n");
    
    SharedData_t data;
    uint32_t last_output_time = 0;
    const uint32_t output_period_ms = 50; /* 20Hz = 50ms */
    
    while (g_running) {
        uint32_t current_time = get_timestamp_ms();
        
        /* 50ms周期输出 */
        if (current_time - last_output_time >= output_period_ms) {
            last_output_time = current_time;
            
            /* 清零数据 */
            memset(&data, 0, sizeof(SharedData_t));
            
            /* 工业级优化：从共享状态缓冲区读取数据 */
            pthread_mutex_lock(&g_shared_state.mutex);
            
            /* 传感器数据 - 从共享缓冲区读取 */
            data.pressure_kg = g_shared_state.pressure_kg;
            data.rope_length_m = g_shared_state.rope_length_m;
            data.encoder_value = g_shared_state.encoder_value;
            data.encoder_angle_deg = g_shared_state.encoder_angle_deg;
            
            /* 电源数据 - 从共享缓冲区读取 */
            data.current_a = g_shared_state.current_a;
            data.voltage_v = g_shared_state.voltage_v;
            
            /* 电机数据 - 从共享缓冲区读取 */
            data.motor_speed_rpm = g_shared_state.motor_speed_rpm;
            data.motor_position = g_shared_state.motor_position_m;
            data.motor_status = g_shared_state.motor_status;
            
            pthread_mutex_unlock(&g_shared_state.mutex);
            
            /* 算法状态 */
            if (g_algorithm_enabled) {
                AlgoStatus_t status;
                gravity_unload_get_status(&g_gravity_ctrl, &status);
                data.algorithm_state = (int32_t)status.state;
                data.algorithm_error = (int32_t)status.error;
                data.clutch_current_ma = 0;
                data.motor_cmd_rpm = 0;
            } else {
                data.algorithm_state = 0;
                data.algorithm_error = 0;
                data.clutch_current_ma = 0;
                data.motor_cmd_rpm = 0;
            }
            
            data.emergency_stop = false;
            
            /* 写入共享内存 */
            if (g_shm_initialized) {
                shm_write_data(&g_shm_mgr, &data);
            }
            
            /* 同时打印到控制台（用于调试） */
            static int console_counter = 0;
            if (++console_counter >= 20) { /* 1Hz控制台输出 */
                console_counter = 0;
                /* 计算线速度 */
                float linear_vel = 0.0f;
                if (g_motor.initialized) {
                    linear_vel = motor_calculate_linear_velocity(&g_motor, data.motor_speed_rpm);
                }
                printf("[DATA] P=%.3fkg Pos=%.3fm I=%.3fA V=%.2fV Motor=%.0frpm Vline=%.3fm/s Algo=%s\n",
                       data.pressure_kg,
                       data.rope_length_m,
                       data.current_a,
                       data.voltage_v,
                       data.motor_speed_rpm,
                       linear_vel,
                       g_algorithm_enabled ? "RUN" : "STOP");
                fflush(stdout);
            }
        }
        
        /* 微秒级延时，保持20Hz精度 */
        usleep(1000); /* 1ms */
    }
    
    printf("[SHM] Data output thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 命令处理线程 - 实时响应上位机命令
 ******************************************************************************/
static void* command_handler_thread(void* arg) {
    (void)arg;
    
    printf("[SHM] Command handler thread started\n");
    
    SharedCommand_t cmd;
    
    while (g_running) {
        if (g_shm_initialized) {
            /* 读取命令 */
            if (shm_read_command(&g_shm_mgr, &cmd) == 0) {
                /* 检查是否有新命令 */
                if (cmd.command_id != g_last_cmd_id && cmd.cmd_type != 0) {
                    g_last_cmd_id = cmd.command_id;
                    
                    printf("[CMD] Received command id=%u type=%d value=%.2f\n",
                           cmd.command_id, cmd.cmd_type, cmd.cmd_value);
                    
                    /* 处理命令 */
                    switch (cmd.cmd_type) {
                        case 1: /* 速度命令 */
                            if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                                /* 确保电机处于循环同步速度模式(CSV) */
                                motor_set_mode(&g_motor, MOTOR_MODE_CSV);
                                /* 发送速度命令（单位：rpm），支持浮点数 */
                                float speed = cmd.cmd_value;
                                ErrorCode_t ret = motor_set_velocity(&g_motor, speed);
                                printf("[CMD] Set motor velocity: %.2f rpm (%s)\n", speed, 
                                       ret == ERR_OK ? "OK" : "FAILED");
                            } else {
                                printf("[CMD] Motor velocity command rejected (enabled=%d, initialized=%d, algo=%d)\n",
                                       g_motor_enabled, g_motor.initialized, g_algorithm_enabled);
                            }
                            break;
                            
                        case 2: /* 位置命令 */
                            if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                                /* 确保电机处于位置模式 */
                                motor_set_mode(&g_motor, MOTOR_MODE_PP);
                                motor_set_position(&g_motor, (int32_t)cmd.cmd_value);
                                printf("[CMD] Set motor position: %.0f\n", cmd.cmd_value);
                            }
                            break;
                            
                        case 3: /* 停止命令 */
                            if (g_motor_enabled && g_motor.initialized) {
                                motor_set_velocity(&g_motor, 0);
                                printf("[CMD] Motor stop\n");
                            }
                            break;
                            
                        case 4: /* 使能命令 */
                            if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                                /* 设置为循环同步速度模式(CSV)后再使能 */
                                motor_set_mode(&g_motor, MOTOR_MODE_CSV);
                                ErrorCode_t ret = motor_enable(&g_motor);
                                printf("[CMD] Motor enable (mode: CSV) - %s\n", 
                                       ret == ERR_OK ? "OK" : "FAILED");
                            }
                            break;
                            
                        case 5: /* 失能命令 */
                            if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                                motor_disable(&g_motor);
                                printf("[CMD] Motor disable\n");
                            }
                            break;
                            
                        case 6: /* 设置电流命令 (A) */
                            {
                                uint16_t current_ma = (uint16_t)(cmd.cmd_value * 1000.0f);
                                ErrorCode_t ret = power_set_current(&g_power, current_ma);
                                if (ret == ERR_OK) {
                                    printf("[CMD] Power current set to %.2f A (%d mA)\n", 
                                           cmd.cmd_value, current_ma);
                                } else {
                                    printf("[CMD] Failed to set power current\n");
                                }
                            }
                            break;
                            
                        case 7: /* 设置电压命令 (V) */
                            {
                                printf("[CMD] Power voltage set to %.2f V (voltage control not implemented)\n", 
                                       cmd.cmd_value);
                                // 电压控制需要根据电源板硬件能力实现
                            }
                            break;
                            
                        default:
                            printf("[CMD] Unknown command type: %d\n", cmd.cmd_type);
                            break;
                    }
                    
                    /* 清空命令 */
                    shm_clear_command(&g_shm_mgr);
                }
                
                /* 处理算法控制命令 */
                if (cmd.algorithm_start && !g_algorithm_enabled) {
                    printf("[CMD] Starting algorithm...\n");
                    /* 算法启动由主循环处理 */
                }
                
                if (cmd.algorithm_stop && g_algorithm_enabled) {
                    printf("[CMD] Stopping algorithm...\n");
                    /* 算法停止由主循环处理 */
                }
            }
        }
        
        /* 10ms轮询 */
        usleep(10000);
    }
    
    printf("[SHM] Command handler thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 监控线程 - 1Hz打印状态
 ******************************************************************************/
static void* monitor_thread(void* arg) {
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
 * 数据收集线程 - 100Hz更新共享缓冲区
 ******************************************************************************/
static void* data_collection_thread(void* arg) {
    (void)arg;
    
    printf("[DATA] Data collection thread started (100Hz)\n");
    
    uint32_t last_time = get_timestamp_ms();
    const uint32_t period_ms = 10; /* 100Hz = 10ms */
    
    while (g_running) {
        uint32_t current_time = get_timestamp_ms();
        
        /* 10ms周期更新 */
        if (current_time - last_time >= period_ms) {
            last_time = current_time;
            
            /* 更新传感器数据到共享缓冲区 */
            SensorData_t encoder_data, pressure_data;
            if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data) == ERR_OK &&
                sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure_data) == ERR_OK) {
                update_sensor_to_buffer(&encoder_data, &pressure_data);
            }
            
            /* 更新电源数据到共享缓冲区 */
            update_power_to_buffer(&g_power);
            
            /* 更新电机数据到共享缓冲区 */
            if (g_motor_enabled && g_motor.initialized) {
                update_motor_to_buffer();
            }
        }
        
        /* 短暂睡眠，避免CPU占用过高 */
        usleep(1000);
    }
    
    printf("[DATA] Data collection thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 主函数
 ******************************************************************************/
int main(int argc, char *argv[]) {
    /* 关键修复：切换到SDK目录运行，确保SDK能找到所有依赖文件 */
    const char* sdk_path = "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/lib";
    if (chdir(sdk_path) != 0) {
        printf("WARNING: Failed to change directory to SDK path: %s\n", sdk_path);
    } else {
        printf("Running from SDK directory: %s\n", sdk_path);
    }
    
    /* 设置LD_LIBRARY_PATH */
    setenv("LD_LIBRARY_PATH", sdk_path, 1);
    
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
    logger_init(&g_logger, NULL, LOG_LEVEL_DEBUG, 1);
    
    printf("========================================\n");
    printf("CANOpNode_Sys v4.0.0 (Gravity Unload)\n");
    printf("NiMotion SDK Integration - Industrial Grade\n");
    printf("Shared Memory Communication Enabled\n");
    printf("========================================\n\n");
    fflush(stdout);
    
    /* ========== 阶段1: 初始化硬件 ========== */
    printf("[INIT] Phase 1: Initializing hardware...\n");
    
    /* 1. 初始化统一传感器管理器 */
    printf("  -> Sensor manager... ");
    fflush(stdout);
    if (sensor_mgr_init(&g_sensor_mgr, ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE) != ERR_OK) {
        printf("WARNING (sensor disabled)\n");
        printf("     ! Sensors will not be available\n");
    } else {
        printf("OK\n");
    }
    
    sensor_mgr_set_encoder_rope_params(&g_sensor_mgr, 100.0f, 4096);
    
    /* 2. 初始化电源板 */
    printf("  -> Power driver... ");
    fflush(stdout);
    if (power_init(&g_power, POWER_UART_DEVICE, POWER_UART_BAUDRATE) != ERR_OK) {
        printf("WARNING (power disabled)\n");
        printf("     ! Power monitoring will not be available\n");
    } else {
        printf("OK\n");
    }
    
    /* 3. 初始化电机 */
    if (g_motor_enabled) {
        printf("  -> Motor driver... ");
        fflush(stdout);
        if (motor_init(&g_motor, MOTOR_NODE_ID, MOTOR_CAN_INTERFACE) != ERR_OK) {
            printf("WARNING (non-critical)\n");
            g_motor_enabled = 0;
        } else {
            printf("OK\n");
            printf("  -> Motor ready (will enable when needed)\n");
        }
    }
    
    /* 4. 启动传感器管理线程 */
    printf("  -> Starting sensor manager thread... ");
    fflush(stdout);
    if (sensor_mgr_start(&g_sensor_mgr) != ERR_OK) {
        printf("WARNING (sensor thread disabled)\n");
    } else {
        printf("OK\n");
        
        /* 等待传感器线程完成首次读取 */
        printf("  -> Waiting for sensors to stabilize... ");
        fflush(stdout);
        sleep(2);
        
        /* 等待编码器数据有效 */
        SensorData_t encoder_data;
        int wait_count = 0;
        while (wait_count < 50) {  /* 最多等待5秒 */
            if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data) == ERR_OK 
                && encoder_data.data_valid) {
                break;
            }
            usleep(100000);
            wait_count++;
        }
        
        if (!encoder_data.data_valid) {
            printf("WARNING (sensor data invalid)\n");
        } else {
            printf("OK\n");
            
            /* 5. 执行校准（必须在传感器线程启动并数据有效后） */
            printf("  -> Calibrating sensors... ");
            fflush(stdout);
            sensor_mgr_encoder_zero_calibration(&g_sensor_mgr);
            usleep(500000);
            sensor_mgr_pressure_tare(&g_sensor_mgr);
            printf("OK\n");
        }
    }
    
    printf("\n");
    
    /* ========== 阶段2: 初始化共享内存 ========== */
    printf("[INIT] Phase 2: Initializing shared memory...\n");
    printf("  -> Creating shared memory segment... ");
    fflush(stdout);
    
    if (shm_init(&g_shm_mgr, true) != 0) {
        printf("FAILED\n");
        printf("[WARNING] Shared memory init failed, continuing without it\n");
    } else {
        g_shm_initialized = 1;
        printf("OK (name=%s, size=%d)\n", SHM_NAME, SHM_SIZE);
    }
    
    /* ========== 阶段3: 系统预检测 ========== */
    printf("\n[INIT] Phase 3: System health check...\n");
    
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
        if (g_shm_initialized) shm_close(&g_shm_mgr);
        return 1;
    }
    
    printf("\n[INIT] All system checks PASSED!\n\n");
    
    /* ========== 阶段4: 启动数据通信线程 ========== */
    printf("[INIT] Phase 4: Starting communication threads...\n");
    
    printf("  -> Starting data collection thread (100Hz)... ");
    fflush(stdout);
    pthread_create(&g_collection_thread_tid, NULL, data_collection_thread, NULL);
    printf("OK\n");
    
    printf("  -> Starting data output thread (20Hz)... ");
    fflush(stdout);
    pthread_create(&g_data_thread_tid, NULL, data_output_thread, NULL);
    printf("OK\n");
    
    printf("  -> Starting command handler thread... ");
    fflush(stdout);
    pthread_create(&g_command_thread_tid, NULL, command_handler_thread, NULL);
    printf("OK\n");
    
    /* ========== 阶段5: 用户确认 ========== */
    printf("\n[INIT] Phase 5: System ready for remote monitoring\n");
    printf("  - Shared Memory: %s\n", g_shm_initialized ? "ACTIVE" : "OFFLINE");
    printf("  - Data Output: 20Hz\n");
    printf("  - Motor Control: Available (manual mode)\n");
    printf("  - Algorithm: Standby\n\n");
    
    printf("Waiting for commands from GravShow...\n");
    printf("Or press Enter to start algorithm mode locally\n");
    printf("Press Ctrl+C to exit\n\n");
    
    /* 非阻塞检查用户输入 */
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; /* 100ms */
    
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    
    int user_input = 0;
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            user_input = 1;
        }
    }
    
    /* ========== 阶段6: 算法模式（可选） ========== */
    if (user_input) {
        printf("\n[INFO] Starting algorithm mode...\n\n");
        g_algorithm_mode = 1;
        
        /* 先使能电机 */
        if (g_motor_enabled) {
            printf("  -> Enabling motor... ");
            fflush(stdout);
            if (motor_enable(&g_motor) != ERR_OK) {
                printf("WARNING (non-critical)\n");
                g_motor_enabled = 0;
            } else {
                printf("OK\n");
            }
        }
        
        /* 初始化算法 */
        if (gravity_unload_init(&g_gravity_ctrl) != 0) {
            printf("[ERROR] Failed to initialize algorithm\n");
            g_running = 0;
        } else {
            /* 启动算法 */
            if (gravity_unload_start(&g_gravity_ctrl) != 0) {
                printf("[ERROR] Failed to start algorithm\n");
                g_running = 0;
            } else {
                g_algorithm_enabled = 1;
                printf("[INIT] Algorithm started successfully\n");
                
                /* 创建监控线程 */
                pthread_create(&g_monitor_thread_tid, NULL, monitor_thread, NULL);
            }
        }
    } else {
        printf("[INFO] Running in manual mode (remote control enabled)\n\n");
    }
    
    printf("========================================\n");
    printf("System RUNNING\n");
    if (g_algorithm_enabled) {
        printf("Mode: ALGORITHM (Gravity Unload)\n");
    } else {
        printf("Mode: MANUAL (Remote Control)\n");
    }
    printf("Press Ctrl+C to stop\n");
    printf("========================================\n\n");
    fflush(stdout);
    
    /* ========== 主循环 ========== */
    while (g_running) {
        /* 算法模式：检查算法状态 */
        if (g_algorithm_enabled) {
            AlgoStatus_t status;
            gravity_unload_get_status(&g_gravity_ctrl, &status);
            
            if (status.state == ALGO_STATE_EMERGENCY_STOP) {
                printf("\n[EMERGENCY] Algorithm stopped due to safety violation!\n");
                break;
            }
            
            if (status.error != ALGO_ERR_NONE && status.error != ALGO_ERR_INVALID_PARAM) {
                printf("\n[WARNING] Algorithm error: %d\n", status.error);
            }
        }
        
        /* 检查共享内存命令中的算法控制 */
        if (g_shm_initialized) {
            SharedCommand_t cmd;
            if (shm_read_command(&g_shm_mgr, &cmd) == 0) {
                if (cmd.algorithm_start && !g_algorithm_enabled) {
                    printf("\n[CMD] Starting algorithm from remote...\n");
                    /* 这里可以添加算法启动逻辑 */
                }
                if (cmd.algorithm_stop && g_algorithm_enabled) {
                    printf("\n[CMD] Stopping algorithm from remote...\n");
                    g_running = 0;
                }
            }
        }
        
        sleep(1);
    }
    
    /* ========== 清理 ========== */
    printf("\n[SHUTDOWN] Stopping system...\n");
    
    /* 停止算法 */
    if (g_algorithm_enabled) {
        printf("  -> Stopping algorithm... ");
        fflush(stdout);
        gravity_unload_stop(&g_gravity_ctrl);
        gravity_unload_deinit(&g_gravity_ctrl);
        printf("OK\n");
        
        /* 等待监控线程结束 */
        pthread_join(g_monitor_thread_tid, NULL);
    }
    
    /* 等待通信线程结束 */
    printf("  -> Waiting for communication threads... ");
    fflush(stdout);
    pthread_join(g_collection_thread_tid, NULL);
    pthread_join(g_data_thread_tid, NULL);
    pthread_join(g_command_thread_tid, NULL);
    printf("OK\n");
    
    /* 停止传感器管理器 */
    printf("  -> Stopping sensor manager... ");
    fflush(stdout);
    sensor_mgr_stop(&g_sensor_mgr);
    printf("OK\n");
    
    /* 打印最终状态 */
    printf("\n");
    sensor_mgr_print_status(&g_sensor_mgr);
    
    /* 反初始化设备 */
    power_deinit(&g_power);
    sensor_mgr_deinit(&g_sensor_mgr);
    
    /* 关闭共享内存 */
    if (g_shm_initialized) {
        printf("  -> Closing shared memory... ");
        fflush(stdout);
        shm_close(&g_shm_mgr);
        printf("OK\n");
    }
    
    printf("\n[System shutdown completed]\n");
    logger_deinit(NULL);
    
    return 0;
}
