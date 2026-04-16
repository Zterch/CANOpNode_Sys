/******************************************************************************
 * @file    power_driver.c
 * @brief   电源板驱动实现 - 占位符
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#include "power_driver.h"
#include "../utils/logger.h"
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

ErrorCode_t power_init(PowerDriver_t *power, const char *device, int baudrate) {
    LOG_WARN(LOG_MODULE_POWER, "Power driver not implemented yet");
    return ERR_OK;
}

void power_deinit(PowerDriver_t *power) {
    LOG_INFO(LOG_MODULE_POWER, "Power driver deinitialized");
}

ErrorCode_t power_set_current(PowerDriver_t *power, uint16_t current_ma) {
    return ERR_OK;
}

ErrorCode_t power_get_current(PowerDriver_t *power, uint16_t *current_ma) {
    *current_ma = 0;
    return ERR_OK;
}

ErrorCode_t power_on(PowerDriver_t *power) {
    return ERR_OK;
}

ErrorCode_t power_off(PowerDriver_t *power) {
    return ERR_OK;
}
