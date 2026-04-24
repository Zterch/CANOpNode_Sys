/******************************************************************************
 * @file    pid_controller.c
 * @brief   PID控制器模块实现
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#include "pid_controller.h"
#include <string.h>

void pid_init(PID_Controller_t *pid, 
              float kp, float ki, float kd,
              float output_min, float output_max,
              float dt) {
    if (pid == NULL) return;
    
    memset(pid, 0, sizeof(PID_Controller_t));
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_limit = output_max; /* 默认积分限幅等于输出上限 */
    pid->dt = dt;
    pid->enable_integral = 1;
    pid->enable_derivative = 1;
    pid->first_run = 1;
}

void pid_reset(PID_Controller_t *pid) {
    if (pid == NULL) return;
    
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_output = 0.0f;
    pid->first_run = 1;
}

float pid_update(PID_Controller_t *pid, float setpoint, float measurement) {
    if (pid == NULL) return 0.0f;
    
    /* 计算误差 */
    float error = setpoint - measurement;
    
    /* 比例项 */
    float p_term = pid->kp * error;
    
    /* 积分项 */
    float i_term = 0.0f;
    if (pid->enable_integral) {
        pid->integral += error * pid->dt;
        
        /* 积分限幅 */
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        } else if (pid->integral < -pid->integral_limit) {
            pid->integral = -pid->integral_limit;
        }
        
        i_term = pid->ki * pid->integral;
    }
    
    /* 微分项 */
    float d_term = 0.0f;
    if (pid->enable_derivative && !pid->first_run) {
        float derivative = (error - pid->last_error) / pid->dt;
        d_term = pid->kd * derivative;
    }
    
    /* 计算输出 */
    float output = p_term + i_term + d_term;
    
    /* 输出限幅 */
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    /* 更新状态 */
    pid->last_error = error;
    pid->last_output = output;
    pid->first_run = 0;
    
    return output;
}

void pid_set_parameters(PID_Controller_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_output_limits(PID_Controller_t *pid, float min, float max) {
    if (pid == NULL) return;
    
    pid->output_min = min;
    pid->output_max = max;
}

void pid_enable_integral(PID_Controller_t *pid, int enable) {
    if (pid == NULL) return;
    
    pid->enable_integral = enable;
    if (!enable) {
        pid->integral = 0.0f;
    }
}

void pid_enable_derivative(PID_Controller_t *pid, int enable) {
    if (pid == NULL) return;
    
    pid->enable_derivative = enable;
}
