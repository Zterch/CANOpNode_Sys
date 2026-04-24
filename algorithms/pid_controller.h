/******************************************************************************
 * @file    pid_controller.h
 * @brief   PID控制器模块
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * PID控制器结构体
 ******************************************************************************/
typedef struct {
    /* PID参数 */
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */
    
    /* 输出限制 */
    float output_min;   /* 输出下限 */
    float output_max;   /* 输出上限 */
    float integral_limit; /* 积分限幅 */
    
    /* 内部状态 */
    float integral;     /* 积分项 */
    float last_error;   /* 上次误差 */
    float last_output;  /* 上次输出 */
    
    /* 配置 */
    int enable_integral;    /* 是否启用积分 */
    int enable_derivative;  /* 是否启用微分 */
    int first_run;          /* 首次运行标志 */
    
    /* 采样时间 */
    float dt;           /* 采样周期 s */
} PID_Controller_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器实例
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_min 输出下限
 * @param output_max 输出上限
 * @param dt 采样周期 s
 */
void pid_init(PID_Controller_t *pid, 
              float kp, float ki, float kd,
              float output_min, float output_max,
              float dt);

/**
 * @brief 重置PID控制器
 * @param pid PID控制器实例
 */
void pid_reset(PID_Controller_t *pid);

/**
 * @brief 更新PID控制器
 * @param pid PID控制器实例
 * @param setpoint 设定值
 * @param measurement 测量值
 * @return 控制输出
 */
float pid_update(PID_Controller_t *pid, float setpoint, float measurement);

/**
 * @brief 设置PID参数
 * @param pid PID控制器实例
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void pid_set_parameters(PID_Controller_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置输出限制
 * @param pid PID控制器实例
 * @param min 输出下限
 * @param max 输出上限
 */
void pid_set_output_limits(PID_Controller_t *pid, float min, float max);

/**
 * @brief 启用/禁用积分项
 * @param pid PID控制器实例
 * @param enable 1=启用, 0=禁用
 */
void pid_enable_integral(PID_Controller_t *pid, int enable);

/**
 * @brief 启用/禁用微分项
 * @param pid PID控制器实例
 * @param enable 1=启用, 0=禁用
 */
void pid_enable_derivative(PID_Controller_t *pid, int enable);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H__ */
