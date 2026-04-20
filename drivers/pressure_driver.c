/******************************************************************************
 * @file    pressure_driver.c
 * @brief   压力计驱动实现 - BSQ-DG-V2数字型变送器 Modbus RTU
 * @author  System Architect
 * @date    2026-04-20
 * @version 1.0.0
 * 
 * 协议说明：
 * - Modbus RTU协议
 * - 支持功能码03（读取保持寄存器）
 * - 读取显示值：寄存器地址0x0000，数量1
 * 
 * 示例：
 *   发送：01 03 00 00 00 01 84 0A
 *   接收：01 03 02 XX XX CRC_L CRC_H
 ******************************************************************************/

#include "pressure_driver.h"
#include "rs485_bus.h"
#include "../utils/logger.h"
#include "../config/system_config.h"

#include <string.h>
#include <unistd.h>

/******************************************************************************
 * 公共接口实现
 ******************************************************************************/

ErrorCode_t pressure_init(PressureDriver_t *pressure, const char *device, 
                          int baudrate, uint8_t slave_addr) {
    (void)device;   /* 串口由RS485总线管理器统一管理 */
    (void)baudrate;
    
    if (pressure == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(pressure, 0, sizeof(PressureDriver_t));
    
    pressure->slave_addr = slave_addr;
    pressure->max_range = PRESSURE_MAX_RANGE;
    pressure->zero_offset = PRESSURE_ZERO_OFFSET;
    
    if (pthread_mutex_init(&pressure->mutex, NULL) != 0) {
        return ERR_GENERAL;
    }
    
    /* 串口由RS485总线管理器统一管理 */
    pressure->fd = -1;
    pressure->initialized = 1;
    pressure->pressure = 0.0f;
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure driver initialized (addr=%d)", slave_addr);
    
    return ERR_OK;
}

void pressure_deinit(PressureDriver_t *pressure) {
    if (pressure == NULL || !pressure->initialized) {
        return;
    }
    
    pthread_mutex_lock(&pressure->mutex);
    pressure->initialized = 0;
    pthread_mutex_unlock(&pressure->mutex);
    
    pthread_mutex_destroy(&pressure->mutex);
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure driver deinitialized");
}

ErrorCode_t pressure_read(PressureDriver_t *pressure, float *value) {
    if (pressure == NULL || !pressure->initialized || value == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    uint8_t request[8];
    uint8_t response[16];
    
    /* 构建Modbus RTU请求帧
     * 读取显示值：寄存器地址0x0000，数量1
     * 发送：01 03 00 00 00 01 CRC_L CRC_H
     */
    request[0] = pressure->slave_addr;      /* 设备地址 */
    request[1] = PRESSURE_MODBUS_FUNC_CODE; /* 功能码: 读取保持寄存器 */
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
    
    /* 接收响应 - 压力传感器响应格式：
     * 地址(1) + 功能码(1) + 字节数(1) + 数据(2) + CRC(2) = 7字节
     */
    int rx_len = 0;
    ret = rs485_bus_receive(response, 16, &rx_len, 50);
    
    pthread_mutex_unlock(bus_mutex);
    
    if (ret != ERR_OK || rx_len < 7) {
        return ERR_TIMEOUT;
    }
    
    /* 验证响应 */
    if (response[0] != pressure->slave_addr) {
        return ERR_COMM_FAIL;
    }
    
    if (response[1] != PRESSURE_MODBUS_FUNC_CODE) {
        return ERR_COMM_FAIL;
    }
    
    if (response[2] != 2) {
        return ERR_COMM_FAIL;
    }
    
    /* 解析压力值 (16位有符号整数, 大端模式)
     * 根据手册，需要根据量程转换
     * 假设传感器输出0-10000对应0-100%量程
     */
    int16_t raw_value = (int16_t)(((uint16_t)response[3] << 8) | (uint16_t)response[4]);
    
    pthread_mutex_lock(&pressure->mutex);
    
    /* 转换为实际压力值（根据实际传感器标定调整） */
    /* 假设传感器输出0-10000对应0-100%量程 */
    pressure->pressure = ((float)raw_value / 10000.0f) * pressure->max_range - pressure->zero_offset;
    
    *value = pressure->pressure;
    
    pthread_mutex_unlock(&pressure->mutex);
    
    return ERR_OK;
}

ErrorCode_t pressure_zero_calibration(PressureDriver_t *pressure) {
    if (pressure == NULL || !pressure->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    float current_value;
    
    /* 读取当前压力值作为零点偏移 */
    if (pressure_read(pressure, &current_value) != ERR_OK) {
        return ERR_COMM_FAIL;
    }
    
    pthread_mutex_lock(&pressure->mutex);
    pressure->zero_offset = current_value;
    pthread_mutex_unlock(&pressure->mutex);
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Zero calibration completed: offset=%.3f", pressure->zero_offset);
    
    return ERR_OK;
}
