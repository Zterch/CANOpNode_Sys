/******************************************************************************
 * @file    encoder_driver.c
 * @brief   编码器驱动实现 - 占位符
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#include "encoder_driver.h"
#include "../utils/logger.h"
#include <string.h>

ErrorCode_t encoder_init(EncoderDriver_t *encoder, const char *device, 
                         int baudrate, uint8_t slave_addr) {
    LOG_WARN(LOG_MODULE_ENCODER, "Encoder driver not implemented yet");
    return ERR_OK;
}

void encoder_deinit(EncoderDriver_t *encoder) {
    LOG_INFO(LOG_MODULE_ENCODER, "Encoder driver deinitialized");
}

ErrorCode_t encoder_read_position(EncoderDriver_t *encoder, int32_t *position) {
    *position = 0;
    return ERR_OK;
}

ErrorCode_t encoder_read_velocity(EncoderDriver_t *encoder, int32_t *velocity) {
    *velocity = 0;
    return ERR_OK;
}
