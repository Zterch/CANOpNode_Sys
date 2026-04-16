/******************************************************************************
 * @file    main.c
 * @brief   CANOpNode_Sys 主程序 - 多设备控制系统
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 * 
 * 系统架构:
 * - 配置层 (config): 系统配置参数
 * - 驱动层 (drivers): 设备驱动程序
 * - 算法层 (algorithms): 控制算法
 * - 工具层 (utils): 日志、线程管理等工具
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

#include "config/system_config.h"
#include "utils/logger.h"
#include "utils/thread_manager.h"
#include "drivers/motor_driver.h"
#include "drivers/power_driver.h"
#include "drivers/encoder_driver.h"
#include "drivers/pressure_driver.h"
#include "algorithms/sine_wave.h"

/******************************************************************************
 * 全局变量
 ******************************************************************************/
static volatile int g_running = 1;          /* 系统运行标志 */
static SystemState_t g_system_state = SYS_STATE_INIT;

/* 设备驱动实例 */
static MotorDriver_t g_motor = {0};
static PowerDriver_t g_power = {0};
static EncoderDriver_t g_encoder = {0};
static PressureDriver_t g_pressure = {0};

/* 线程管理器 */
static ThreadManager_t g_thread_mgr = {0};

/* 正弦波生成器 */
static SineWaveGenerator_t g_sine_gen = {0};

/******************************************************************************
 * 信号处理
 ******************************************************************************/
void signal_handler(int sig) {
    LOG_INFO(LOG_MODULE_SYS, "Received signal %d, shutting down...", sig);
    g_running = 0;
    g_system_state = SYS_STATE_SHUTDOWN;
}

/******************************************************************************
 * 控制线程 - 核心控制循环
 ******************************************************************************/
void* control_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    struct timespec ts_start, ts_end;
    double cycle_time_ms;
    
    LOG_INFO(LOG_MODULE_SYS, "Control thread started");
    
    /* 初始化正弦波生成器 */
    sine_wave_init(&g_sine_gen, MOTOR_CSV_AMPLITUDE, MOTOR_CSV_FREQUENCY, 
                   0.0, 0.0, SYS_CONTROL_PERIOD_MS / 1000.0);
    
    /* 使能电机 */
    if (motor_enable(&g_motor) != ERR_OK) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to enable motor");
        ctrl->state = THREAD_STATE_ERROR;
        return NULL;
    }
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        clock_gettime(CLOCK_MONOTONIC, &ts_start);
        
        /* 生成正弦波速度指令 */
        double velocity_cmd = sine_wave_generate(&g_sine_gen);
        
        /* 发送速度指令到电机 */
        if (motor_set_velocity(&g_motor, (int32_t)velocity_cmd) != ERR_OK) {
            LOG_WARN(LOG_MODULE_MOTOR, "Failed to set motor velocity");
        }
        
        /* 更新电机状态 */
        motor_update_state(&g_motor);
        
        /* 计算周期时间 */
        clock_gettime(CLOCK_MONOTONIC, &ts_end);
        cycle_time_ms = (ts_end.tv_sec - ts_start.tv_sec) * 1000.0 +
                        (ts_end.tv_nsec - ts_start.tv_nsec) / 1000000.0;
        
        thread_update_stats(ctrl, cycle_time_ms);
        
        /* 精确延时 */
        usleep(SYS_CONTROL_PERIOD_MS * 1000);
    }
    
    /* 停止电机 */
    motor_set_velocity(&g_motor, 0);
    usleep(100000);
    motor_disable(&g_motor);
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_SYS, "Control thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 数据采集线程
 ******************************************************************************/
void* data_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    struct timespec ts_start, ts_end;
    double cycle_time_ms;
    int32_t velocity, position;
    
    LOG_INFO(LOG_MODULE_SYS, "Data acquisition thread started");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        clock_gettime(CLOCK_MONOTONIC, &ts_start);
        
        /* 读取电机数据 */
        if (motor_get_velocity(&g_motor, &velocity) == ERR_OK &&
            motor_get_position(&g_motor, &position) == ERR_OK) {
            /* 数据可用于后续处理 */
        }
        
        /* TODO: 读取编码器数据（待实现） */
        /* encoder_read_position(&g_encoder, &encoder_pos); */
        
        /* TODO: 读取压力计数据（待实现） */
        /* pressure_read(&g_pressure, &pressure_val); */
        
        /* 计算周期时间 */
        clock_gettime(CLOCK_MONOTONIC, &ts_end);
        cycle_time_ms = (ts_end.tv_sec - ts_start.tv_sec) * 1000.0 +
                        (ts_end.tv_nsec - ts_start.tv_nsec) / 1000000.0;
        
        thread_update_stats(ctrl, cycle_time_ms);
        
        usleep(SYS_DATA_RECORD_PERIOD_MS * 1000);
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_SYS, "Data acquisition thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 日志线程
 ******************************************************************************/
void* log_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    int32_t velocity, position;
    
    LOG_INFO(LOG_MODULE_SYS, "Log thread started");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        /* 读取电机状态 */
        if (motor_get_velocity(&g_motor, &velocity) == ERR_OK &&
            motor_get_position(&g_motor, &position) == ERR_OK) {
            
            LOG_INFO(LOG_MODULE_MOTOR, "Vel=%6d Pos=%10d State=%s",
                     velocity, position, 
                     motor_get_state_string(g_motor.state));
        }
        
        /* 打印线程统计信息 */
        thread_mgr_print_status(&g_thread_mgr);
        
        usleep(SYS_LOG_PERIOD_MS * 1000);
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_SYS, "Log thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 系统初始化
 ******************************************************************************/
ErrorCode_t system_init(void) {
    ErrorCode_t ret;
    
    LOG_INFO(LOG_MODULE_SYS, "========================================");
    LOG_INFO(LOG_MODULE_SYS, "System: %s v%s", SYSTEM_NAME, SYSTEM_VERSION);
    LOG_INFO(LOG_MODULE_SYS, "========================================");
    
    /* 初始化线程管理器 */
    ret = thread_mgr_init(&g_thread_mgr);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to initialize thread manager");
        return ret;
    }
    
    /* 初始化电机驱动 */
    LOG_INFO(LOG_MODULE_MOTOR, "Initializing motor driver...");
    ret = motor_init(&g_motor, MOTOR_NODE_ID, MOTOR_CAN_INTERFACE);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to initialize motor driver");
        return ret;
    }
    LOG_INFO(LOG_MODULE_MOTOR, "Motor driver initialized (Node ID=%d)", MOTOR_NODE_ID);
    
    /* TODO: 初始化电源板驱动（待实现） */
    /* power_init(&g_power, POWER_UART_DEVICE, POWER_UART_BAUDRATE); */
    
    /* TODO: 初始化编码器驱动（待实现） */
    /* encoder_init(&g_encoder, ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE, ENCODER_SLAVE_ADDR); */
    
    /* TODO: 初始化压力计驱动（待实现） */
    /* pressure_init(&g_pressure, PRESSURE_UART_DEVICE, PRESSURE_UART_BAUDRATE, PRESSURE_SLAVE_ADDR); */
    
    g_system_state = SYS_STATE_READY;
    LOG_INFO(LOG_MODULE_SYS, "System initialization completed");
    
    return ERR_OK;
}

/******************************************************************************
 * 系统反初始化
 ******************************************************************************/
void system_deinit(void) {
    LOG_INFO(LOG_MODULE_SYS, "Shutting down system...");
    
    /* 停止所有线程 */
    thread_mgr_stop_all(&g_thread_mgr);
    
    /* 反初始化设备 */
    motor_deinit(&g_motor);
    power_deinit(&g_power);
    encoder_deinit(&g_encoder);
    pressure_deinit(&g_pressure);
    
    /* 反初始化线程管理器 */
    thread_mgr_deinit(&g_thread_mgr);
    
    g_system_state = SYS_STATE_SHUTDOWN;
    LOG_INFO(LOG_MODULE_SYS, "System shutdown completed");
}

/******************************************************************************
 * 创建系统线程
 ******************************************************************************/
ErrorCode_t create_system_threads(void) {
    ErrorCode_t ret;
    ThreadConfig_t config;
    
    /* 创建控制线程 */
    config.type = THREAD_TYPE_CONTROL;
    config.name = "Control";
    config.priority = THREAD_PRIORITY_HIGH;
    config.period_ms = SYS_CONTROL_PERIOD_MS;
    config.func = control_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create control thread");
        return ret;
    }
    
    /* 创建数据采集线程 */
    config.type = THREAD_TYPE_DATA;
    config.name = "DataAcq";
    config.priority = THREAD_PRIORITY_NORMAL;
    config.period_ms = SYS_DATA_RECORD_PERIOD_MS;
    config.func = data_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create data thread");
        return ret;
    }
    
    /* 创建日志线程 */
    config.type = THREAD_TYPE_LOG;
    config.name = "Logger";
    config.priority = THREAD_PRIORITY_LOW;
    config.period_ms = SYS_LOG_PERIOD_MS;
    config.func = log_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create log thread");
        return ret;
    }
    
    LOG_INFO(LOG_MODULE_SYS, "All system threads created");
    return ERR_OK;
}

/******************************************************************************
 * 主函数
 ******************************************************************************/
int main(int argc, char *argv[]) {
    ErrorCode_t ret;
    
    /* 注册信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* 初始化日志系统 */
    ret = logger_init(&g_logger, "system.log", LOG_LEVEL_INFO, 1);
    if (ret != ERR_OK) {
        fprintf(stderr, "Failed to initialize logger\n");
        return 1;
    }
    
    /* 系统初始化 */
    ret = system_init();
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "System initialization failed");
        logger_deinit(&g_logger);
        return 1;
    }
    
    /* 创建系统线程 */
    ret = create_system_threads();
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create system threads");
        system_deinit();
        logger_deinit(&g_logger);
        return 1;
    }
    
    /* 主循环 */
    g_system_state = SYS_STATE_RUNNING;
    LOG_INFO(LOG_MODULE_SYS, "System is running, press Ctrl+C to stop...");
    
    /* 运行指定时间后自动停止 */
    sleep(DEMO_RUN_TIME_S);
    
    LOG_INFO(LOG_MODULE_SYS, "Demo time (%d seconds) completed", DEMO_RUN_TIME_S);
    g_running = 0;
    
    /* 系统反初始化 */
    system_deinit();
    logger_deinit(&g_logger);
    
    printf("\nSystem exited normally.\n");
    return 0;
}
