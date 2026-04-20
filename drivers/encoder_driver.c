/******************************************************************************
 * @file    encoder_driver.c
 * @brief   编码器驱动实现 - RS485接口绝对值编码器 (Modbus RTU)
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 * 
 * 协议说明：
 * - Modbus RTU协议，功能码0x03
 * - 寄存器地址0x0000，读取编码器单圈值
 * - 波特率9600，8N1
 ******************************************************************************/

#include "encoder_driver.h"
#include "rs485_bus.h"
#include "../utils/logger.h"

#include <string.h>
#include <unistd.h>
#include <time.h>

/******************************************************************************
 * 公共接口实现
 ******************************************************************************/

ErrorCode_t encoder_init(EncoderDriver_t *encoder, const char *device, 
                         int baudrate, uint8_t slave_addr) {
    if (encoder == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(encoder, 0, sizeof(EncoderDriver_t));
    
    encoder->slave_addr = slave_addr;
    encoder->baudrate = baudrate;
    encoder->resolution = ENCODER_RESOLUTION;
    strncpy(encoder->device, device, sizeof(encoder->device) - 1);
    
    if (pthread_mutex_init(&encoder->mutex, NULL) != 0) {
        return ERR_GENERAL;
    }
    
    /* 串口由RS485总线管理器统一管理，这里只记录配置 */
    encoder->fd = -1;
    encoder->initialized = 1;
    encoder->position = 0;
    encoder->velocity = 0;
    encoder->last_position = 0;
    encoder->timestamp = clock();
    encoder->last_time = encoder->timestamp;
    
    LOG_INFO(LOG_MODULE_ENCODER, "Encoder driver initialized (addr=%d)", slave_addr);
    
    return ERR_OK;
}

void encoder_deinit(EncoderDriver_t *encoder) {
    if (encoder == NULL || !encoder->initialized) {
        return;
    }
    
    pthread_mutex_lock(&encoder->mutex);
    encoder->initialized = 0;
    pthread_mutex_unlock(&encoder->mutex);
    
    pthread_mutex_destroy(&encoder->mutex);
    
    LOG_INFO(LOG_MODULE_ENCODER, "Encoder driver deinitialized");
}

ErrorCode_t encoder_read_position(EncoderDriver_t *encoder, int32_t *position) {
    if (encoder == NULL || !encoder->initialized || position == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    uint8_t request[8];
    uint8_t response[16];
    
    /* 构建Modbus RTU请求帧 */
    request[0] = encoder->slave_addr;       /* 设备地址 */
    request[1] = ENCODER_MODBUS_FUNC_CODE;  /* 功能码: 读取保持寄存器 */
    request[2] = 0x00;                      /* 寄存器地址高字节 */
    request[3] = 0x00;                      /* 寄存器地址低字节 */
    request[4] = 0x00;                      /* 读取数量高字节 */
    request[5] = 0x01;                      /* 读取数量低字节: 1个寄存器 */
    
    /* 计算CRC */
    uint16_t crc = crc16_modbus(request, 6);
    request[6] = crc & 0xFF;                /* CRC低字节 */
    request[7] = (crc >> 8) & 0xFF;         /* CRC高字节 */
    
    /* 获取RS485总线互斥锁 */
    pthread_mutex_t *bus_mutex = rs485_bus_get_mutex();
    pthread_mutex_lock(bus_mutex);
    
    /* 清空接收缓冲区 */
    rs485_bus_flush();
    
    /* Modbus RTU帧间隔: 3.5字符时间 @9600bps ≈ 4ms */
    usleep(5000);
    
    /* 发送请求 */
    ErrorCode_t ret = rs485_bus_send(request, 8, 50);
    if (ret != ERR_OK) {
        pthread_mutex_unlock(bus_mutex);
        return ERR_COMM_FAIL;
    }
    
    /* 等待设备响应 */
    usleep(10000);
    
    /* 接收响应 - 标准Modbus响应格式：
     * 地址(1) + 功能码(1) + 字节数(1) + 数据(2) + CRC(2) = 7字节
     */
    int rx_len = 0;
    ret = rs485_bus_receive(response, 16, &rx_len, 50);
    
    pthread_mutex_unlock(bus_mutex);
    
    if (ret != ERR_OK || rx_len < 5) {
        return ERR_TIMEOUT;
    }
    
    /* 验证响应 */
    if (response[0] != encoder->slave_addr) {
        return ERR_COMM_FAIL;
    }
    
    if (response[1] != ENCODER_MODBUS_FUNC_CODE) {
        return ERR_COMM_FAIL;
    }
    
    /* 解析编码器数值 (16位无符号整数, 大端模式) */
    uint16_t raw_value = ((uint16_t)response[3] << 8) | (uint16_t)response[4];
    
    pthread_mutex_lock(&encoder->mutex);
    
    encoder->last_position = encoder->position;
    encoder->last_time = encoder->timestamp;
    encoder->position = (int32_t)raw_value;
    encoder->timestamp = clock();
    
    /* 计算速度 */
    clock_t dt = encoder->timestamp - encoder->last_time;
    if (dt > 0) {
        encoder->velocity = ((encoder->position - encoder->last_position) * 1000) / dt;
    }
    
    *position = encoder->position;
    
    pthread_mutex_unlock(&encoder->mutex);
    
    return ERR_OK;
}

ErrorCode_t encoder_read_velocity(EncoderDriver_t *encoder, int32_t *velocity) {
    if (encoder == NULL || !encoder->initialized || velocity == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&encoder->mutex);
    *velocity = encoder->velocity;
    pthread_mutex_unlock(&encoder->mutex);
    
    return ERR_OK;
}

ErrorCode_t encoder_get_last_position(EncoderDriver_t *encoder, int32_t *position, clock_t *timestamp) {
    if (encoder == NULL || !encoder->initialized || position == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&encoder->mutex);
    *position = encoder->position;
    if (timestamp != NULL) {
        *timestamp = encoder->timestamp;
    }
    pthread_mutex_unlock(&encoder->mutex);
    
    return ERR_OK;
}
