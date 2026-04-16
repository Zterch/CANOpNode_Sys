/******************************************************************************
 * @file    encoder_driver.h
 * @brief   编码器驱动 - RS485接口绝对值编码器
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#ifndef __ENCODER_DRIVER_H__
#define __ENCODER_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 编码器数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    char device[32];            /* 串口设备 */
    int baudrate;               /* 波特率 */
    uint8_t slave_addr;         /* RS485设备地址 */
    int fd;                     /* 串口文件描述符 */
    
    /* 数据 */
    int32_t position;           /* 位置值 */
    int32_t velocity;           /* 速度值 */
    uint32_t resolution;        /* 分辨率 */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    int initialized;
} EncoderDriver_t;

/******************************************************************************
 * 函数声明 - 待实现
 ******************************************************************************/

/**
 * @brief 初始化编码器驱动
 * @param encoder 编码器驱动结构指针
 * @param device 串口设备
 * @param baudrate 波特率
 * @param slave_addr 设备地址
 * @return ErrorCode_t
 */
ErrorCode_t encoder_init(EncoderDriver_t *encoder, const char *device, 
                         int baudrate, uint8_t slave_addr);

/**
 * @brief 反初始化编码器驱动
 * @param encoder 编码器驱动结构指针
 */
void encoder_deinit(EncoderDriver_t *encoder);

/**
 * @brief 读取编码器位置
 * @param encoder 编码器驱动结构指针
 * @param position 位置输出指针
 * @return ErrorCode_t
 */
ErrorCode_t encoder_read_position(EncoderDriver_t *encoder, int32_t *position);

/**
 * @brief 读取编码器速度
 * @param encoder 编码器驱动结构指针
 * @param velocity 速度输出指针
 * @return ErrorCode_t
 */
ErrorCode_t encoder_read_velocity(EncoderDriver_t *encoder, int32_t *velocity);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_DRIVER_H__ */
