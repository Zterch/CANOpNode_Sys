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
#include "drivers/rs485_bus.h"
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

/* 编码器全局数据（用于线程间共享） */
static int32_t g_encoder_position = 0;
static int32_t g_encoder_velocity = 0;
static pthread_mutex_t g_encoder_data_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 压力传感器全局数据 */
static float g_pressure_value = 0.0f;
static pthread_mutex_t g_pressure_data_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 电源板全局数据 */
static uint16_t g_power_current = 0;        /* 实际电流 (mA) */
static uint16_t g_power_voltage = 0;        /* 实际电压 (mV) */
static pthread_mutex_t g_power_data_mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************
 * 信号处理
 ******************************************************************************/
void signal_handler(int sig) {
    LOG_INFO(LOG_MODULE_SYS, "Received signal %d, shutting down...", sig);
    g_running = 0;
    g_system_state = SYS_STATE_SHUTDOWN;
}

/******************************************************************************
 * 控制线程 - 核心控制循环 (仅电机控制)
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
 * 编码器数据采集线程 - 50Hz读取
 ******************************************************************************/
void* encoder_data_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    struct timespec ts_start, ts_end;
    double cycle_time_ms;
    int32_t position, velocity;
    
    LOG_INFO(LOG_MODULE_ENCODER, "Encoder data thread started (50Hz)");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        clock_gettime(CLOCK_MONOTONIC, &ts_start);
        
        /* 读取编码器位置 (非阻塞，带超时) */
        if (encoder_read_position(&g_encoder, &position) == ERR_OK) {
            /* 读取成功，获取速度 */
            encoder_read_velocity(&g_encoder, &velocity);
            
            /* 更新全局数据 */
            pthread_mutex_lock(&g_encoder_data_mutex);
            g_encoder_position = position;
            g_encoder_velocity = velocity;
            pthread_mutex_unlock(&g_encoder_data_mutex);
            
            g_encoder.read_count++;
        } else {
            /* 读取失败，增加错误计数 */
            g_encoder.error_count++;
        }
        
        /* 计算周期时间 */
        clock_gettime(CLOCK_MONOTONIC, &ts_end);
        cycle_time_ms = (ts_end.tv_sec - ts_start.tv_sec) * 1000.0 +
                        (ts_end.tv_nsec - ts_start.tv_nsec) / 1000000.0;
        
        thread_update_stats(ctrl, cycle_time_ms);
        
        /* 50Hz = 20ms周期 */
        int sleep_us = ENCODER_READ_PERIOD_MS * 1000 - (int)(cycle_time_ms * 1000);
        if (sleep_us > 0) {
            usleep(sleep_us);
        }
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_ENCODER, "Encoder data thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 编码器打印线程 - 2Hz打印
 ******************************************************************************/
void* encoder_print_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    int32_t position, velocity;
    uint32_t last_read_count = 0;
    uint32_t last_error_count = 0;
    
    LOG_INFO(LOG_MODULE_ENCODER, "Encoder print thread started (2Hz)");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        /* 获取编码器数据 */
        pthread_mutex_lock(&g_encoder_data_mutex);
        position = g_encoder_position;
        velocity = g_encoder_velocity;
        pthread_mutex_unlock(&g_encoder_data_mutex);
        
        /* 计算读取成功率 */
        uint32_t reads = g_encoder.read_count - last_read_count;
        uint32_t errors = g_encoder.error_count - last_error_count;
        last_read_count = g_encoder.read_count;
        last_error_count = g_encoder.error_count;
        
        float success_rate = (reads + errors > 0) ? 
            (100.0f * reads / (reads + errors)) : 0.0f;
        
        /* 计算角度 */
        float angle = encoder_position_to_angle((uint16_t)position, ENCODER_RESOLUTION);
        
        /* 打印编码器数据 */
        LOG_INFO(LOG_MODULE_ENCODER, 
            "Pos=%5d Angle=%6.2f° | Reads: %u, Errors: %u, Success: %.1f%%",
            position, angle, reads, errors, success_rate);
        
        /* 2Hz = 500ms周期 */
        usleep(ENCODER_PRINT_PERIOD_MS * 1000);
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_ENCODER, "Encoder print thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 压力传感器数据采集线程 - 10Hz读取
 ******************************************************************************/
void* pressure_data_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    struct timespec ts_start, ts_end;
    double cycle_time_ms;
    float pressure;
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure data thread started (50Hz)");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        clock_gettime(CLOCK_MONOTONIC, &ts_start);
        
        /* 读取压力值 */
        if (pressure_read(&g_pressure, &pressure) == ERR_OK) {
            /* 更新全局数据 */
            pthread_mutex_lock(&g_pressure_data_mutex);
            g_pressure_value = pressure;
            pthread_mutex_unlock(&g_pressure_data_mutex);
        }
        /* 读取失败不阻塞，继续下一次读取 */
        
        /* 计算周期时间 */
        clock_gettime(CLOCK_MONOTONIC, &ts_end);
        cycle_time_ms = (ts_end.tv_sec - ts_start.tv_sec) * 1000.0 +
                        (ts_end.tv_nsec - ts_start.tv_nsec) / 1000000.0;
        
        thread_update_stats(ctrl, cycle_time_ms);
        
        /* 50Hz = 20ms周期 */
        int sleep_us = PRESSURE_READ_PERIOD_MS * 1000 - (int)(cycle_time_ms * 1000);
        if (sleep_us > 0) {
            usleep(sleep_us);
        }
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure data thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 压力传感器打印线程 - 1Hz打印
 ******************************************************************************/
void* pressure_print_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    float pressure;
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure print thread started (1Hz)");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        /* 获取压力数据 */
        pthread_mutex_lock(&g_pressure_data_mutex);
        pressure = g_pressure_value;
        pthread_mutex_unlock(&g_pressure_data_mutex);
        
        /* 打印压力数据 */
        LOG_INFO(LOG_MODULE_PRESSURE, "Pressure=%7.3f kg", pressure);
        
        /* 1Hz = 1000ms周期 */
        usleep(PRESSURE_PRINT_PERIOD_MS * 1000);
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure print thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 电源板数据采集线程 - 50Hz读取
 ******************************************************************************/
void* power_data_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    struct timespec ts_start, ts_end;
    double cycle_time_ms;
    uint16_t current, voltage;
    
    LOG_INFO(LOG_MODULE_POWER, "Power data thread started (50Hz)");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        clock_gettime(CLOCK_MONOTONIC, &ts_start);
        
        /* 读取电流和电压 */
        if (power_get_status(&g_power, &current, &voltage) == ERR_OK) {
            /* 更新全局数据 */
            pthread_mutex_lock(&g_power_data_mutex);
            g_power_current = current;
            g_power_voltage = voltage;
            pthread_mutex_unlock(&g_power_data_mutex);
            
            g_power.read_count++;
        } else {
            /* 读取失败，增加错误计数 */
            g_power.error_count++;
        }
        
        /* 计算周期时间 */
        clock_gettime(CLOCK_MONOTONIC, &ts_end);
        cycle_time_ms = (ts_end.tv_sec - ts_start.tv_sec) * 1000.0 +
                        (ts_end.tv_nsec - ts_start.tv_nsec) / 1000000.0;
        
        thread_update_stats(ctrl, cycle_time_ms);
        
        /* 50Hz = 20ms周期 */
        int sleep_us = POWER_READ_PERIOD_MS * 1000 - (int)(cycle_time_ms * 1000);
        if (sleep_us > 0) {
            usleep(sleep_us);
        }
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_POWER, "Power data thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 电源板打印线程 - 1Hz打印
 ******************************************************************************/
void* power_print_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    uint16_t current, voltage;
    uint32_t last_read_count = 0;
    
    LOG_INFO(LOG_MODULE_POWER, "Power print thread started (1Hz)");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    while (!thread_should_exit(ctrl) && g_running) {
        /* 获取电源板数据 */
        pthread_mutex_lock(&g_power_data_mutex);
        current = g_power_current;
        voltage = g_power_voltage;
        pthread_mutex_unlock(&g_power_data_mutex);
        
        /* 计算读取次数 */
        uint32_t reads = g_power.read_count - last_read_count;
        last_read_count = g_power.read_count;
        
        /* 打印电源板数据 */
        LOG_INFO(LOG_MODULE_POWER, 
            "Voltage=%5.2fV Current=%5.2fA | Reads: %u",
            voltage / 1000.0f, current / 1000.0f, reads);
        
        /* 1Hz = 1000ms周期 */
        usleep(POWER_PRINT_PERIOD_MS * 1000);
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
    LOG_INFO(LOG_MODULE_POWER, "Power print thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 电源板测试线程 - 5秒后设置电流为0.6A
 ******************************************************************************/
void* power_test_thread(void* arg) {
    ThreadCtrl_t *ctrl = (ThreadCtrl_t*)arg;
    
    LOG_INFO(LOG_MODULE_POWER, "Power test thread started");
    LOG_INFO(LOG_MODULE_POWER, "Default current: 0.44A, will change to 0.6A after 5 seconds");
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    /* 等待5秒 */
    sleep(5);
    
    if (!g_running) {
        ctrl->state = THREAD_STATE_STOPPED;
        return NULL;
    }
    
    /* 设置电流为0.6A (600mA) */
    LOG_INFO(LOG_MODULE_POWER, "Setting current to 0.6A...");
    if (power_set_current(&g_power, 600) == ERR_OK) {
        LOG_INFO(LOG_MODULE_POWER, "Current successfully set to 0.6A");
    } else {
        LOG_ERROR(LOG_MODULE_POWER, "Failed to set current to 0.6A");
    }
    
    ctrl->state = THREAD_STATE_STOPPED;
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
    
    /* 初始化RS485总线（编码器和压力传感器共用） */
    LOG_INFO(LOG_MODULE_SYS, "Initializing RS485 bus...");
    ret = rs485_bus_init(ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to initialize RS485 bus");
        return ret;
    }
    LOG_INFO(LOG_MODULE_SYS, "RS485 bus initialized");
    
    /* 初始化电机驱动 - 编码器模式下可跳过 */
    LOG_INFO(LOG_MODULE_MOTOR, "Initializing motor driver...");
    ret = motor_init(&g_motor, MOTOR_NODE_ID, MOTOR_CAN_INTERFACE);
    if (ret != ERR_OK) {
        LOG_WARN(LOG_MODULE_MOTOR, "Failed to initialize motor driver (may be OK for encoder-only mode)");
        /* 不返回错误，继续初始化编码器 */
    } else {
        LOG_INFO(LOG_MODULE_MOTOR, "Motor driver initialized (Node ID=%d)", MOTOR_NODE_ID);
    }
    
    /* 初始化编码器驱动 */
    LOG_INFO(LOG_MODULE_ENCODER, "Initializing encoder driver...");
    ret = encoder_init(&g_encoder, ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE, ENCODER_SLAVE_ADDR);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_ENCODER, "Failed to initialize encoder driver");
        rs485_bus_deinit();
        motor_deinit(&g_motor);
        return ret;
    }
    LOG_INFO(LOG_MODULE_ENCODER, "Encoder driver initialized (Addr=%d)", ENCODER_SLAVE_ADDR);
    
    /* 初始化压力传感器驱动 */
    LOG_INFO(LOG_MODULE_PRESSURE, "Initializing pressure driver...");
    ret = pressure_init(&g_pressure, PRESSURE_UART_DEVICE, PRESSURE_UART_BAUDRATE, PRESSURE_SLAVE_ADDR);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_PRESSURE, "Failed to initialize pressure driver");
        encoder_deinit(&g_encoder);
        rs485_bus_deinit();
        motor_deinit(&g_motor);
        return ret;
    }
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure driver initialized (Addr=%d)", PRESSURE_SLAVE_ADDR);
    
    /* 初始化电源板驱动 */
    LOG_INFO(LOG_MODULE_POWER, "Initializing power driver...");
    ret = power_init(&g_power, POWER_UART_DEVICE, POWER_UART_BAUDRATE);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_POWER, "Failed to initialize power driver");
        pressure_deinit(&g_pressure);
        encoder_deinit(&g_encoder);
        rs485_bus_deinit();
        motor_deinit(&g_motor);
        return ret;
    }
    LOG_INFO(LOG_MODULE_POWER, "Power driver initialized (default 0.44A)");
    
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
    pressure_deinit(&g_pressure);
    encoder_deinit(&g_encoder);
    
    /* 反初始化RS485总线 */
    rs485_bus_deinit();
    
    /* 销毁互斥锁 */
    pthread_mutex_destroy(&g_encoder_data_mutex);
    pthread_mutex_destroy(&g_pressure_data_mutex);
    pthread_mutex_destroy(&g_power_data_mutex);
    
    /* 反初始化线程管理器 */
    thread_mgr_deinit(&g_thread_mgr);
    
    g_system_state = SYS_STATE_SHUTDOWN;
    LOG_INFO(LOG_MODULE_SYS, "System shutdown completed");
}

/******************************************************************************
 * 创建系统线程
 ******************************************************************************/
ErrorCode_t create_system_threads(int motor_enabled) {
    ErrorCode_t ret;
    ThreadConfig_t config;
    
    /* 创建编码器数据采集线程 (50Hz) */
    config.type = THREAD_TYPE_DATA;
    config.name = "EncoderData";
    config.priority = THREAD_PRIORITY_NORMAL;
    config.period_ms = ENCODER_READ_PERIOD_MS;
    config.func = encoder_data_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create encoder data thread");
        return ret;
    }
    
    /* 创建编码器打印线程 (2Hz) */
    config.type = THREAD_TYPE_LOG;
    config.name = "EncoderPrint";
    config.priority = THREAD_PRIORITY_LOW;
    config.period_ms = ENCODER_PRINT_PERIOD_MS;
    config.func = encoder_print_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create encoder print thread");
        return ret;
    }
    
    /* 创建压力传感器数据采集线程 (10Hz) */
    config.type = THREAD_TYPE_DATA;
    config.name = "PressureData";
    config.priority = THREAD_PRIORITY_NORMAL;
    config.period_ms = PRESSURE_READ_PERIOD_MS;
    config.func = pressure_data_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create pressure data thread");
        return ret;
    }
    
    /* 创建压力传感器打印线程 (2Hz) */
    config.type = THREAD_TYPE_LOG;
    config.name = "PressurePrint";
    config.priority = THREAD_PRIORITY_LOW;
    config.period_ms = PRESSURE_PRINT_PERIOD_MS;
    config.func = pressure_print_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create pressure print thread");
        return ret;
    }
    
    /* 创建电源板数据采集线程 (50Hz) */
    config.type = THREAD_TYPE_DATA;
    config.name = "PowerData";
    config.priority = THREAD_PRIORITY_NORMAL;
    config.period_ms = POWER_READ_PERIOD_MS;
    config.func = power_data_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create power data thread");
        return ret;
    }
    
    /* 创建电源板打印线程 (1Hz) */
    config.type = THREAD_TYPE_LOG;
    config.name = "PowerPrint";
    config.priority = THREAD_PRIORITY_LOW;
    config.period_ms = POWER_PRINT_PERIOD_MS;
    config.func = power_print_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create power print thread");
        return ret;
    }
    
    /* 创建电源板测试线程（5秒后设置0.6A） */
    config.type = THREAD_TYPE_CONTROL;
    config.name = "PowerTest";
    config.priority = THREAD_PRIORITY_NORMAL;
    config.period_ms = 0;  /* 一次性任务 */
    config.func = power_test_thread;
    config.arg = NULL;
    
    ret = thread_mgr_create(&g_thread_mgr, &config);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create power test thread");
        return ret;
    }
    
    /* 如果电机使能，创建控制线程 */
    if (motor_enabled) {
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
    }
    
    LOG_INFO(LOG_MODULE_SYS, "All system threads created (motor %s)", 
             motor_enabled ? "enabled" : "disabled");
    return ERR_OK;
}

/******************************************************************************
 * 主函数
 ******************************************************************************/
int main(int argc, char *argv[]) {
    ErrorCode_t ret;
    int motor_enabled = 1;
    
    /* 解析命令行参数 */
    if (argc > 1) {
        if (strcmp(argv[1], "--no-motor") == 0 || strcmp(argv[1], "-n") == 0) {
            motor_enabled = 0;
            printf("Motor control disabled (encoder-only mode)\n");
        }
    }
    
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
    ret = create_system_threads(motor_enabled);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to create system threads");
        system_deinit();
        logger_deinit(&g_logger);
        return 1;
    }
    
    /* 主循环 */
    g_system_state = SYS_STATE_RUNNING;
    if (motor_enabled) {
        LOG_INFO(LOG_MODULE_SYS, "System is running with motor control, press Ctrl+C to stop...");
    } else {
        LOG_INFO(LOG_MODULE_SYS, "System is running (full sensor mode), press Ctrl+C to stop...");
    }
    
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
