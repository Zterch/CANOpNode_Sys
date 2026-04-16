/******************************************************************************
 * @file    pressure_driver.h
 * @brief   压力计驱动 - RS485接口压力传感器
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#ifndef __PRESSURE_DRIVER_H__
#define __PRESSURE_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 压力计数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    char device[32];            /* 串口设备 */
    int baudrate;               /* 波特率 */
    uint8_t slave_addr;         /* RS485设备地址 */
    int fd;                     /* 串口文件描述符 */
    
    /* 数据 */
    float pressure;             /* 压力值 */
    float max_range;            /* 最大量程 */
    float zero_offset;          /* 零点偏移 */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    int initialized;
} PressureDriver_t;

/******************************************************************************
 * 函数声明 - 待实现
 ******************************************************************************/

/**
 * @brief 初始化压力计驱动
 * @param pressure 压力计驱动结构指针
 * @param device 串口设备
 * @param baudrate 波特率
 * @param slave_addr 设备地址
 * @return ErrorCode_t
 */
ErrorCode_t pressure_init(PressureDriver_t *pressure, const char *device, 
                          int baudrate, uint8_t slave_addr);

/**
 * @brief 反初始化压力计驱动
 * @param pressure 压力计驱动结构指针
 */
void pressure_deinit(PressureDriver_t *pressure);

/**
 * @brief 读取压力值
 * @param pressure 压力计驱动结构指针
 * @param value 压力输出指针
 * @return ErrorCode_t
 */
ErrorCode_t pressure_read(PressureDriver_t *pressure, float *value);

/**
 * @brief 零点校准
 * @param pressure 压力计驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t pressure_zero_calibration(PressureDriver_t *pressure);

#ifdef __cplusplus
}
#endif

#endif /* __PRESSURE_DRIVER_H__ */
