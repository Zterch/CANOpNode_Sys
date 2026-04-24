/******************************************************************************
 * @file    system_check.h
 * @brief   系统预检测模块 - 算法启动前的系统健康检查
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#ifndef __SYSTEM_CHECK_H__
#define __SYSTEM_CHECK_H__

#include <stdint.h>
#include "algorithm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 检测结果枚举
 ******************************************************************************/
typedef enum {
    CHECK_RESULT_PASS = 0,
    CHECK_RESULT_FAIL_SENSOR,
    CHECK_RESULT_FAIL_MOTOR,
    CHECK_RESULT_FAIL_POWER,
    CHECK_RESULT_FAIL_COMM,
    CHECK_RESULT_FAIL_SAFETY
} SystemCheckResult_t;

/******************************************************************************
 * 检测项结构
 ******************************************************************************/
typedef struct {
    const char *name;
    int (*check_func)(void);
    const char *description;
    int critical;  /* 1=关键项，必须通过 */
} CheckItem_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 执行完整的系统预检测
 * @return 检测结果
 */
SystemCheckResult_t system_check_run(void);

/**
 * @brief 检测传感器状态
 * @return 0=正常, -1=异常
 */
int system_check_sensors(void);

/**
 * @brief 检测电机状态
 * @return 0=正常, -1=异常
 */
int system_check_motor(void);

/**
 * @brief 检测电源板状态
 * @return 0=正常, -1=异常
 */
int system_check_power(void);

/**
 * @brief 检测通信状态
 * @return 0=正常, -1=异常
 */
int system_check_communication(void);

/**
 * @brief 检测安全系统
 * @return 0=正常, -1=异常
 */
int system_check_safety(void);

/**
 * @brief 请求用户确认
 * @return 1=确认继续, 0=取消
 */
int system_request_user_confirmation(void);

/**
 * @brief 将检测结果转换为字符串
 * @param result 检测结果
 * @return 结果字符串
 */
const char* system_check_result_to_string(SystemCheckResult_t result);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_CHECK_H__ */
