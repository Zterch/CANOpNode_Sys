/******************************************************************************
 * @file    rs485_bus.h
 * @brief   RS485总线管理器头文件
 * @author  System Architect
 * @date    2026-04-20
 * @version 1.0.0
 ******************************************************************************/

#ifndef __RS485_BUS_H__
#define __RS485_BUS_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * RS485总线数据结构
 ******************************************************************************/
typedef struct {
    char device[32];            /* 串口设备 */
    int baudrate;               /* 波特率 */
    int fd;                     /* 串口文件描述符 */
    pthread_mutex_t mutex;      /* 线程互斥锁 */
    int initialized;
} RS485Bus_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief CRC16计算函数（标准Modbus）
 * @param pbuf 数据缓冲区
 * @param num 数据长度
 * @return CRC16值
 */
uint16_t crc16_modbus(const uint8_t *pbuf, uint8_t num);

/**
 * @brief 初始化RS485总线
 * @param device 串口设备
 * @param baudrate 波特率
 * @return ErrorCode_t
 */
ErrorCode_t rs485_bus_init(const char *device, int baudrate);

/**
 * @brief 反初始化RS485总线
 */
void rs485_bus_deinit(void);

/**
 * @brief 获取串口文件描述符
 * @return 文件描述符
 */
int rs485_bus_get_fd(void);

/**
 * @brief 获取互斥锁指针
 * @return 互斥锁指针
 */
pthread_mutex_t* rs485_bus_get_mutex(void);

/**
 * @brief 通过RS485总线发送数据
 * @param data 数据缓冲区
 * @param len 数据长度
 * @param timeout_ms 超时时间(ms)
 * @return ErrorCode_t
 */
ErrorCode_t rs485_bus_send(const uint8_t *data, int len, int timeout_ms);

/**
 * @brief 通过RS485总线接收数据
 * @param data 数据缓冲区
 * @param max_len 最大接收长度
 * @param received_len 实际接收长度输出
 * @param timeout_ms 超时时间(ms)
 * @return ErrorCode_t
 */
ErrorCode_t rs485_bus_receive(uint8_t *data, int max_len, int *received_len, int timeout_ms);

/**
 * @brief 清空串口缓冲区
 */
void rs485_bus_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_BUS_H__ */
