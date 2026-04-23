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
#include <stdio.h>
#include <math.h>

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
    
    /* 初始化绳子长度参数 */
    encoder->rope_drum_diameter = 100.0f;    /* 默认卷筒直径50mm */
    encoder->rope_length_per_turn = M_PI * encoder->rope_drum_diameter; /* 每圈周长 */
    encoder->rope_length_base = 0.0f;       /* 默认基准长度0 */
    encoder->rope_length_current = 0.0f;
    encoder->multi_turn_zero_offset = 0;    /* 零点偏移 */
    
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

/******************************************************************************
 * 虚拟多圈值读取功能
 ******************************************************************************/

ErrorCode_t encoder_read_multi_turn(EncoderDriver_t *encoder, uint32_t *multi_turn_value) {
    if (encoder == NULL || !encoder->initialized || multi_turn_value == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    uint8_t request[8];
    uint8_t response[16];
    
    /* 构建Modbus RTU请求帧 - 读取2个寄存器 (4字节) */
    request[0] = encoder->slave_addr;       /* 设备地址 */
    request[1] = ENCODER_MODBUS_FUNC_CODE;  /* 功能码: 读取保持寄存器 */
    request[2] = 0x00;                      /* 寄存器地址高字节 */
    request[3] = 0x00;                      /* 寄存器地址低字节 */
    request[4] = 0x00;                      /* 读取数量高字节 */
    request[5] = 0x02;                      /* 读取数量低字节: 2个寄存器 (4字节) */
    
    /* 计算CRC */
    uint16_t crc = crc16_modbus(request, 6);
    request[6] = crc & 0xFF;                /* CRC低字节 */
    request[7] = (crc >> 8) & 0xFF;         /* CRC高字节 */
    
    /* 获取RS485总线互斥锁 */
    pthread_mutex_t *bus_mutex = rs485_bus_get_mutex();
    pthread_mutex_lock(bus_mutex);
    
    /* 清空接收缓冲区 */
    rs485_bus_flush();
    
    /* Modbus RTU帧间隔 */
    usleep(5000);
    
    /* 发送请求 */
    ErrorCode_t ret = rs485_bus_send(request, 8, 50);
    if (ret != ERR_OK) {
        pthread_mutex_unlock(bus_mutex);
        return ERR_COMM_FAIL;
    }
    
    /* 等待设备响应 */
    usleep(15000);
    
    /* 接收响应 - 读取2个寄存器的响应格式：
     * 地址(1) + 功能码(1) + 字节数(1) + 数据(4) + CRC(2) = 9字节
     */
    int rx_len = 0;
    ret = rs485_bus_receive(response, 16, &rx_len, 50);
    
    pthread_mutex_unlock(bus_mutex);
    
    if (ret != ERR_OK || rx_len < 7) {
        LOG_ERROR(LOG_MODULE_ENCODER, "Multi-turn read timeout or error (rx_len=%d)", rx_len);
        return ERR_TIMEOUT;
    }
    
    /* 验证响应 */
    if (response[0] != encoder->slave_addr) {
        LOG_ERROR(LOG_MODULE_ENCODER, "Multi-turn read: invalid slave addr (0x%02X)", response[0]);
        return ERR_COMM_FAIL;
    }
    
    if (response[1] != ENCODER_MODBUS_FUNC_CODE) {
        LOG_ERROR(LOG_MODULE_ENCODER, "Multi-turn read: invalid function code (0x%02X)", response[1]);
        return ERR_COMM_FAIL;
    }
    
    /* 打印原始数据用于调试 */
    LOG_INFO(LOG_MODULE_ENCODER, "Multi-turn raw response (len=%d): %02X %02X %02X %02X %02X %02X %02X %02X %02X",
              rx_len,
              response[0], response[1], response[2], response[3],
              response[4], response[5], response[6], response[7], response[8]);
    
    /* 检查字节数 - 可能是2字节(16位)或4字节(32位) */
    uint8_t byte_count = response[2];
    if (byte_count != 0x04 && byte_count != 0x02) {
        LOG_ERROR(LOG_MODULE_ENCODER, "Multi-turn read: invalid byte count (%d)", byte_count);
        return ERR_COMM_FAIL;
    }
    
    /* 解析多圈值 */
    uint32_t raw_value;
    if (byte_count == 0x04) {
        /* 32位数据: 高字(2字节) + 低字(2字节) */
        raw_value = ((uint32_t)response[3] << 24) |
                    ((uint32_t)response[4] << 16) |
                    ((uint32_t)response[5] << 8) |
                    ((uint32_t)response[6]);
    } else {
        /* 16位数据 (单圈值) */
        raw_value = ((uint32_t)response[3] << 8) |
                    ((uint32_t)response[4]);
    }
    
    pthread_mutex_lock(&encoder->mutex);
    encoder->multi_turn_value = raw_value;
    /* 计算多圈角度 */
    encoder->multi_turn_angle = (float)raw_value * 360.0f / (float)encoder->resolution;
    *multi_turn_value = raw_value;
    pthread_mutex_unlock(&encoder->mutex);
    
    LOG_DEBUG(LOG_MODULE_ENCODER, "Multi-turn value: %lu (%.2f deg)", 
              (unsigned long)raw_value, encoder->multi_turn_angle);
    
    return ERR_OK;
}

/******************************************************************************
 * 绳子长度计算和记忆功能
 ******************************************************************************/

ErrorCode_t encoder_set_rope_params(EncoderDriver_t *encoder, float drum_diameter, float length_base) {
    if (encoder == NULL || !encoder->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    if (drum_diameter <= 0) {
        LOG_ERROR(LOG_MODULE_ENCODER, "Invalid drum diameter: %.2f", drum_diameter);
        return ERR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&encoder->mutex);
    encoder->rope_drum_diameter = drum_diameter;
    encoder->rope_length_per_turn = M_PI * drum_diameter;
    encoder->rope_length_base = length_base;
    pthread_mutex_unlock(&encoder->mutex);
    
    LOG_INFO(LOG_MODULE_ENCODER, "Rope params set: drum=%.2fmm, per_turn=%.2fmm, base=%.2fmm",
             drum_diameter, encoder->rope_length_per_turn, length_base);
    
    return ERR_OK;
}

ErrorCode_t encoder_zero_calibration(EncoderDriver_t *encoder) {
    if (encoder == NULL || !encoder->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 读取当前多圈值作为零点偏移 */
    uint32_t current_value;
    ErrorCode_t ret = encoder_read_multi_turn(encoder, &current_value);
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_ENCODER, "Zero calibration failed: cannot read multi-turn");
        return ret;
    }
    
    pthread_mutex_lock(&encoder->mutex);
    encoder->multi_turn_zero_offset = current_value;
    pthread_mutex_unlock(&encoder->mutex);
    
    LOG_INFO(LOG_MODULE_ENCODER, "Zero calibration done: offset=%lu", (unsigned long)current_value);
    
    return ERR_OK;
}

ErrorCode_t encoder_calc_rope_length(EncoderDriver_t *encoder, float *length_mm) {
    if (encoder == NULL || !encoder->initialized || length_mm == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    /* 读取当前多圈值 */
    uint32_t current_value;
    ErrorCode_t ret = encoder_read_multi_turn(encoder, &current_value);
    if (ret != ERR_OK) {
        return ret;
    }
    
    pthread_mutex_lock(&encoder->mutex);
    
    /* 计算相对脉冲数 - 使用有符号减法避免溢出 */
    int64_t relative_pulses = (int64_t)current_value - (int64_t)encoder->multi_turn_zero_offset;
    float relative_turns = (float)relative_pulses / (float)encoder->resolution;
    
    /* 计算当前绳子长度 */
    encoder->rope_length_current = encoder->rope_length_base + 
                                   (relative_turns * encoder->rope_length_per_turn);
    
    *length_mm = encoder->rope_length_current;
    
    pthread_mutex_unlock(&encoder->mutex);
    
    return ERR_OK;
}

ErrorCode_t encoder_save_rope_length(EncoderDriver_t *encoder, const char *filename) {
    if (encoder == NULL || !encoder->initialized || filename == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    /* 先读取当前多圈值 */
    uint32_t current_value;
    ErrorCode_t ret = encoder_read_multi_turn(encoder, &current_value);
    if (ret != ERR_OK) {
        return ret;
    }
    
    pthread_mutex_lock(&encoder->mutex);
    
    /* 计算当前绳长 - 注意：relative是脉冲数，需要除以分辨率得到圈数 */
    int64_t relative_pulses = (int64_t)current_value - (int64_t)encoder->multi_turn_zero_offset;
    float relative_turns = (float)relative_pulses / (float)encoder->resolution;
    float current_length = encoder->rope_length_base + (relative_turns * encoder->rope_length_per_turn);
    
    /* 保存当前长度作为新的基准长度 */
    encoder->rope_length_base = current_length;
    encoder->rope_length_current = current_length;
    
    /* 更新零点偏移为当前多圈值 */
    encoder->multi_turn_zero_offset = current_value;
    encoder->multi_turn_value = current_value;
    
    /* 保存到文件 */
    FILE *fp = fopen(filename, "w");
    if (fp == NULL) {
        pthread_mutex_unlock(&encoder->mutex);
        LOG_ERROR(LOG_MODULE_ENCODER, "Failed to open file for saving: %s", filename);
        return ERR_GENERAL;
    }
    
    fprintf(fp, "# Encoder Rope Length Data\n");
    fprintf(fp, "# Saved at: %s\n", __TIME__);
    fprintf(fp, "BASE_LENGTH_MM=%.4f\n", encoder->rope_length_base);
    fprintf(fp, "ZERO_OFFSET=%lu\n", (unsigned long)encoder->multi_turn_zero_offset);
    fprintf(fp, "DRUM_DIAMETER=%.4f\n", encoder->rope_drum_diameter);
    fprintf(fp, "ENCODER_RESOLUTION=%lu\n", (unsigned long)encoder->resolution);
    
    fclose(fp);
    
    pthread_mutex_unlock(&encoder->mutex);
    
    LOG_INFO(LOG_MODULE_ENCODER, "Rope length saved to %s: %.2f mm", filename, current_length);
    
    return ERR_OK;
}

ErrorCode_t encoder_load_rope_length(EncoderDriver_t *encoder, const char *filename) {
    if (encoder == NULL || !encoder->initialized || filename == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    FILE *fp = fopen(filename, "r");
    if (fp == NULL) {
        LOG_WARN(LOG_MODULE_ENCODER, "No saved data found: %s, using defaults", filename);
        return ERR_OK;  /* 文件不存在不是错误，使用默认值 */
    }
    
    float base_length = 0.0f;
    unsigned long zero_offset_temp = 0;
    float drum_diameter = 50.0f;
    unsigned long resolution_temp = 4096;
    
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        /* 跳过注释行 */
        if (line[0] == '#' || line[0] == '\n') continue;
        
        /* 解析键值对 - 使用临时变量避免类型不匹配 */
        if (sscanf(line, "BASE_LENGTH_MM=%f", &base_length) == 1) continue;
        if (sscanf(line, "ZERO_OFFSET=%lu", &zero_offset_temp) == 1) continue;
        if (sscanf(line, "DRUM_DIAMETER=%f", &drum_diameter) == 1) continue;
        if (sscanf(line, "ENCODER_RESOLUTION=%lu", &resolution_temp) == 1) continue;
    }
    
    fclose(fp);
    
    pthread_mutex_lock(&encoder->mutex);
    encoder->rope_length_base = base_length;
    encoder->multi_turn_zero_offset = (uint32_t)zero_offset_temp;
    encoder->rope_drum_diameter = drum_diameter;
    encoder->rope_length_per_turn = M_PI * drum_diameter;
    encoder->resolution = (uint32_t)resolution_temp;
    pthread_mutex_unlock(&encoder->mutex);
    
    LOG_INFO(LOG_MODULE_ENCODER, "Rope length loaded from %s: base=%.2fmm, offset=%lu",
             filename, base_length, (unsigned long)zero_offset_temp);
    
    return ERR_OK;
}

ErrorCode_t encoder_get_multi_turn_angle(EncoderDriver_t *encoder, float *angle_deg) {
    if (encoder == NULL || !encoder->initialized || angle_deg == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    /* 读取当前多圈值 */
    uint32_t current_value;
    ErrorCode_t ret = encoder_read_multi_turn(encoder, &current_value);
    if (ret != ERR_OK) {
        return ret;
    }
    
    pthread_mutex_lock(&encoder->mutex);
    *angle_deg = encoder->multi_turn_angle;
    pthread_mutex_unlock(&encoder->mutex);
    
    return ERR_OK;
}
