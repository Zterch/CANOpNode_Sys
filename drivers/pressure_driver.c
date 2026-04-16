/******************************************************************************
 * @file    pressure_driver.c
 * @brief   压力计驱动实现 - 占位符
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#include "pressure_driver.h"
#include "../utils/logger.h"
#include <string.h>

ErrorCode_t pressure_init(PressureDriver_t *pressure, const char *device, 
                          int baudrate, uint8_t slave_addr) {
    LOG_WARN(LOG_MODULE_PRESSURE, "Pressure driver not implemented yet");
    return ERR_OK;
}

void pressure_deinit(PressureDriver_t *pressure) {
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure driver deinitialized");
}

ErrorCode_t pressure_read(PressureDriver_t *pressure, float *value) {
    *value = 0.0f;
    return ERR_OK;
}

ErrorCode_t pressure_zero_calibration(PressureDriver_t *pressure) {
    return ERR_OK;
}
