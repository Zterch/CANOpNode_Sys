/******************************************************************************
 * @file    power_driver.h
 * @brief   电源板驱动 - UART/TTL串口控制电流输出
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#ifndef __POWER_DRIVER_H__
#define __POWER_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 电源板状态定义
 ******************************************************************************/
typedef enum {
    POWER_STATE_INIT = 0,       /* 初始化 */
    POWER_STATE_OFF,            /* 关闭 */
    POWER_STATE_ON,             /* 开启 */
    POWER_STATE_FAULT,          /* 故障 */
    POWER_STATE_UNKNOWN         /* 未知 */
} PowerState_t;

/******************************************************************************
 * 电源板数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    char device[32];            /* 串口设备 */
    int baudrate;               /* 波特率 */
    int fd;                     /* 串口文件描述符 */
    
    /* 状态信息 */
    PowerState_t state;         /* 电源状态 */
    uint16_t current_setpoint;  /* 设定电流 (mA) */
    uint16_t actual_current;    /* 实际电流 (mA) */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    int initialized;
} PowerDriver_t;

/******************************************************************************
 * 函数声明 - 待实现
 ******************************************************************************/

/**
 * @brief 初始化电源板驱动
 * @param power 电源驱动结构指针
 * @param device 串口设备
 * @param baudrate 波特率
 * @return ErrorCode_t
 */
ErrorCode_t power_init(PowerDriver_t *power, const char *device, int baudrate);

/**
 * @brief 反初始化电源板驱动
 * @param power 电源驱动结构指针
 */
void power_deinit(PowerDriver_t *power);

/**
 * @brief 设置输出电流
 * @param power 电源驱动结构指针
 * @param current_ma 电流值 (mA)
 * @return ErrorCode_t
 */
ErrorCode_t power_set_current(PowerDriver_t *power, uint16_t current_ma);

/**
 * @brief 读取实际电流
 * @param power 电源驱动结构指针
 * @param current_ma 电流输出指针
 * @return ErrorCode_t
 */
ErrorCode_t power_get_current(PowerDriver_t *power, uint16_t *current_ma);

/**
 * @brief 开启电源输出
 * @param power 电源驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t power_on(PowerDriver_t *power);

/**
 * @brief 关闭电源输出
 * @param power 电源驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t power_off(PowerDriver_t *power);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_DRIVER_H__ */
