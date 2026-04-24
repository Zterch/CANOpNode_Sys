/******************************************************************************
 * @file    safety_monitor.c
 * @brief   安全监控模块实现
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#include "safety_monitor.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

void safety_monitor_init(SafetyMonitor_t *monitor) {
    if (monitor == NULL) return;
    
    memset(monitor, 0, sizeof(SafetyMonitor_t));
    monitor->status = SAFETY_STATUS_OK;
    monitor->error_code = ALGO_ERR_NONE;
    strcpy(monitor->error_msg, "OK");
}

void safety_monitor_update(SafetyMonitor_t *monitor, 
                           const SensorDataFiltered_t *filtered_data,
                           const ControlOutput_t *control_output,
                           float motor_speed) {
    if (monitor == NULL || filtered_data == NULL || control_output == NULL) return;
    
    /* 更新当前值 */
    monitor->pressure_kg = filtered_data->pressure_kg;
    monitor->pressure_rate_kg_s = filtered_data->pressure_derivative;
    monitor->position_m = filtered_data->position_m;
    monitor->velocity_m_s = filtered_data->velocity_m_s;
    monitor->clutch_current_mA = control_output->clutch_current_mA;
    monitor->motor_speed = motor_speed;
}

SafetyStatus_t safety_check(SafetyMonitor_t *monitor) {
    if (monitor == NULL) return SAFETY_STATUS_ERROR;
    
    /* 如果已经紧急停止，直接返回 */
    if (monitor->emergency_stop) {
        monitor->status = SAFETY_STATUS_EMERGENCY;
        return SAFETY_STATUS_EMERGENCY;
    }
    
    SafetyStatus_t new_status = SAFETY_STATUS_OK;
    
    /* 检查1: 压力范围 */
    if (monitor->pressure_kg < SAFETY_PRESSURE_MIN_KG || 
        monitor->pressure_kg > SAFETY_PRESSURE_MAX_KG) {
        new_status = SAFETY_STATUS_ERROR;
        monitor->error_code = ALGO_ERR_SAFETY_VIOLATION;
        snprintf(monitor->error_msg, sizeof(monitor->error_msg), 
                 "Pressure out of range: %.2f kg (limit: %.2f to %.2f)",
                 monitor->pressure_kg, SAFETY_PRESSURE_MIN_KG, SAFETY_PRESSURE_MAX_KG);
        monitor->consecutive_errors++;
    }
    /* 检查2: 压力变化率 */
    else if (fabsf(monitor->pressure_rate_kg_s) > SAFETY_PRESSURE_RATE_MAX_KG_S) {
        new_status = SAFETY_STATUS_WARNING;
        snprintf(monitor->error_msg, sizeof(monitor->error_msg),
                 "Pressure rate too high: %.2f kg/s", monitor->pressure_rate_kg_s);
    }
    /* 检查3: 位置范围 */
    else if (monitor->position_m < SAFETY_POSITION_MIN_M || 
             monitor->position_m > SAFETY_POSITION_MAX_M) {
        new_status = SAFETY_STATUS_ERROR;
        monitor->error_code = ALGO_ERR_SAFETY_VIOLATION;
        snprintf(monitor->error_msg, sizeof(monitor->error_msg),
                 "Position out of range: %.2f m", monitor->position_m);
        monitor->consecutive_errors++;
    }
    /* 检查4: 速度限制 */
    else if (fabsf(monitor->velocity_m_s) > SAFETY_SPEED_MAX_M_S) {
        new_status = SAFETY_STATUS_WARNING;
        snprintf(monitor->error_msg, sizeof(monitor->error_msg),
                 "Speed too high: %.2f m/s", monitor->velocity_m_s);
    }
    /* 检查5: 电机速度 */
    else if (fabsf(monitor->motor_speed) > SAFETY_MOTOR_SPEED_MAX) {
        new_status = SAFETY_STATUS_ERROR;
        monitor->error_code = ALGO_ERR_SAFETY_VIOLATION;
        snprintf(monitor->error_msg, sizeof(monitor->error_msg),
                 "Motor speed too high: %.0f", monitor->motor_speed);
        monitor->consecutive_errors++;
    }
    /* 检查6: 离合器电流 */
    else if (monitor->clutch_current_mA < SAFETY_CLUTCH_CURRENT_MIN_MA || 
             monitor->clutch_current_mA > SAFETY_CLUTCH_CURRENT_MAX_MA) {
        new_status = SAFETY_STATUS_ERROR;
        monitor->error_code = ALGO_ERR_SAFETY_VIOLATION;
        snprintf(monitor->error_msg, sizeof(monitor->error_msg),
                 "Clutch current out of range: %.0f mA", monitor->clutch_current_mA);
        monitor->consecutive_errors++;
    }
    else {
        /* 所有检查通过 */
        monitor->consecutive_errors = 0;
        strcpy(monitor->error_msg, "OK");
    }
    
    /* 连续错误次数过多，触发紧急停止 */
    if (monitor->consecutive_errors >= 10) {
        safety_trigger_emergency_stop(monitor, "Too many consecutive errors");
        new_status = SAFETY_STATUS_EMERGENCY;
    }
    
    monitor->status = new_status;
    return new_status;
}

void safety_trigger_emergency_stop(SafetyMonitor_t *monitor, const char *reason) {
    if (monitor == NULL) return;
    
    monitor->emergency_stop = 1;
    monitor->status = SAFETY_STATUS_EMERGENCY;
    monitor->error_code = ALGO_ERR_SAFETY_VIOLATION;
    snprintf(monitor->error_msg, sizeof(monitor->error_msg), 
             "EMERGENCY STOP: %s", reason ? reason : "Unknown");
}

void safety_clear_emergency_stop(SafetyMonitor_t *monitor) {
    if (monitor == NULL) return;
    
    monitor->emergency_stop = 0;
    monitor->status = SAFETY_STATUS_OK;
    monitor->error_code = ALGO_ERR_NONE;
    monitor->consecutive_errors = 0;
    strcpy(monitor->error_msg, "OK");
}

const char* safety_status_to_string(SafetyStatus_t status) {
    switch (status) {
        case SAFETY_STATUS_OK: return "OK";
        case SAFETY_STATUS_WARNING: return "WARNING";
        case SAFETY_STATUS_ERROR: return "ERROR";
        case SAFETY_STATUS_EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

int safety_is_allowed(const SafetyMonitor_t *monitor) {
    if (monitor == NULL) return 0;
    return (monitor->status == SAFETY_STATUS_OK) && !monitor->emergency_stop;
}
