/******************************************************************************
 * @file    safety_monitor.h
 * @brief   安全监控模块 - 实时监测系统安全状态
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#ifndef __SAFETY_MONITOR_H__
#define __SAFETY_MONITOR_H__

#include <stdint.h>
#include "algorithm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 安全监控状态
 ******************************************************************************/
typedef enum {
    SAFETY_STATUS_OK = 0,
    SAFETY_STATUS_WARNING,
    SAFETY_STATUS_ERROR,
    SAFETY_STATUS_EMERGENCY
} SafetyStatus_t;

/******************************************************************************
 * 安全监控器结构体
 ******************************************************************************/
typedef struct {
    /* 当前值 */
    float pressure_kg;
    float pressure_rate_kg_s;
    float position_m;
    float velocity_m_s;
    float clutch_current_mA;
    float motor_speed;
    
    /* 上次值（用于计算变化率） */
    float last_pressure_kg;
    float last_clutch_current_mA;
    uint32_t last_time_ms;
    
    /* 状态 */
    SafetyStatus_t status;
    AlgoError_t error_code;
    char error_msg[128];
    
    /* 计数器 */
    uint32_t violation_count;
    uint32_t consecutive_errors;
    
    /* 紧急停止标志 */
    int emergency_stop;
} SafetyMonitor_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化安全监控器
 * @param monitor 安全监控器实例
 */
void safety_monitor_init(SafetyMonitor_t *monitor);

/**
 * @brief 更新安全监控器输入
 * @param monitor 安全监控器实例
 * @param filtered_data 滤波后的传感器数据
 * @param control_output 控制输出
 */
void safety_monitor_update(SafetyMonitor_t *monitor, 
                           const SensorDataFiltered_t *filtered_data,
                           const ControlOutput_t *control_output,
                           float motor_speed);

/**
 * @brief 执行安全检查
 * @param monitor 安全监控器实例
 * @return 安全状态
 */
SafetyStatus_t safety_check(SafetyMonitor_t *monitor);

/**
 * @brief 触发紧急停止
 * @param monitor 安全监控器实例
 * @param reason 停止原因
 */
void safety_trigger_emergency_stop(SafetyMonitor_t *monitor, const char *reason);

/**
 * @brief 清除紧急停止状态
 * @param monitor 安全监控器实例
 */
void safety_clear_emergency_stop(SafetyMonitor_t *monitor);

/**
 * @brief 获取安全状态字符串
 * @param status 安全状态
 * @return 状态字符串
 */
const char* safety_status_to_string(SafetyStatus_t status);

/**
 * @brief 检查是否允许运行
 * @param monitor 安全监控器实例
 * @return 1=允许, 0=不允许
 */
int safety_is_allowed(const SafetyMonitor_t *monitor);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_MONITOR_H__ */
