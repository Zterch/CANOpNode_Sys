/******************************************************************************
 * @file    motor_driver.c
 * @brief   CANopen电机驱动实现 - Nimotion伺服电机控制
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#include "motor_driver.h"
#include "../utils/logger.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/******************************************************************************
 * CANopen对象字典索引
 ******************************************************************************/
#define OD_INDEX_CONTROL_WORD       0x6040
#define OD_INDEX_STATUS_WORD        0x6041
#define OD_INDEX_MODE_OF_OPERATION  0x6060
#define OD_INDEX_MODE_DISPLAY       0x6061
#define OD_INDEX_POSITION_ACTUAL    0x6064
#define OD_INDEX_VELOCITY_ACTUAL    0x606C
#define OD_INDEX_TARGET_VELOCITY    0x60FF
#define OD_INDEX_TARGET_POSITION    0x607A

/******************************************************************************
 * CANopen COB-ID定义
 ******************************************************************************/
#define COB_ID_NMT          0x000
#define COB_ID_SDO_TX_BASE  0x580
#define COB_ID_SDO_RX_BASE  0x600

/******************************************************************************
 * CiA 402控制字
 ******************************************************************************/
#define CONTROL_WORD_SHUTDOWN           0x0006
#define CONTROL_WORD_SWITCH_ON          0x0007
#define CONTROL_WORD_ENABLE_OPERATION   0x000F
#define CONTROL_WORD_DISABLE_VOLTAGE    0x0000

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static int can_socket_init(const char *interface);
static int can_send_frame(int sock, uint32_t can_id, uint8_t *data, uint8_t dlc);
static int can_receive_frame(int sock, uint32_t *can_id, uint8_t *data, uint8_t *dlc, int timeout_ms);
static int sdo_read(MotorDriver_t *motor, uint16_t index, uint8_t subindex, uint32_t *value);
static int sdo_write(MotorDriver_t *motor, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size);
static int nmt_send(MotorDriver_t *motor, uint8_t command);

/******************************************************************************
 * 函数实现
 ******************************************************************************/

static int can_socket_init(const char *interface) {
    int sock;
    struct sockaddr_can addr;
    struct ifreq ifr;

    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to create CAN socket");
        return -1;
    }

    strcpy(ifr.ifr_name, interface);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to get interface index for %s", interface);
        close(sock);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to bind CAN socket");
        close(sock);
        return -1;
    }

    return sock;
}

static int can_send_frame(int sock, uint32_t can_id, uint8_t *data, uint8_t dlc) {
    struct can_frame frame;
    
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        return -1;
    }
    
    return 0;
}

static int can_receive_frame(int sock, uint32_t *can_id, uint8_t *data, uint8_t *dlc, int timeout_ms) {
    struct can_frame frame;
    fd_set rdfs;
    struct timeval tv;
    int ret;

    FD_ZERO(&rdfs);
    FD_SET(sock, &rdfs);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(sock + 1, &rdfs, NULL, NULL, &tv);
    if (ret <= 0) {
        return -1;
    }

    if (read(sock, &frame, sizeof(struct can_frame)) < 0) {
        return -1;
    }

    *can_id = frame.can_id;
    *dlc = frame.can_dlc;
    memcpy(data, frame.data, frame.can_dlc);
    
    return 0;
}

static int sdo_read(MotorDriver_t *motor, uint16_t index, uint8_t subindex, uint32_t *value) {
    uint8_t data[8];
    uint32_t rx_id;
    uint8_t rx_dlc;
    uint8_t rx_data[8];
    
    memset(data, 0, 8);
    data[0] = 0x40;
    data[1] = index & 0xFF;
    data[2] = (index >> 8) & 0xFF;
    data[3] = subindex;
    
    if (can_send_frame(motor->socket_fd, COB_ID_SDO_RX_BASE + motor->node_id, data, 8) < 0) {
        return -1;
    }
    
    for (int i = 0; i < 100; i++) {
        if (can_receive_frame(motor->socket_fd, &rx_id, rx_data, &rx_dlc, 100) == 0) {
            if (rx_id == (COB_ID_SDO_TX_BASE + motor->node_id)) {
                if ((rx_data[0] & 0xE0) == 0x80) {
                    return -1;
                }
                if (rx_data[0] & 0x02) {
                    int n = (rx_data[0] >> 2) & 0x03;
                    *value = 0;
                    for (int j = 0; j < (4 - n); j++) {
                        *value |= (rx_data[4 + j] << (j * 8));
                    }
                    return 0;
                }
            }
        }
    }
    
    return -1;
}

static int sdo_write(MotorDriver_t *motor, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size) {
    uint8_t data[8];
    uint32_t rx_id;
    uint8_t rx_dlc;
    uint8_t rx_data[8];
    
    memset(data, 0, 8);
    data[0] = 0x23 | ((4 - size) << 2);
    data[1] = index & 0xFF;
    data[2] = (index >> 8) & 0xFF;
    data[3] = subindex;
    data[4] = value & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = (value >> 16) & 0xFF;
    data[7] = (value >> 24) & 0xFF;
    
    if (can_send_frame(motor->socket_fd, COB_ID_SDO_RX_BASE + motor->node_id, data, 8) < 0) {
        return -1;
    }
    
    for (int i = 0; i < 100; i++) {
        if (can_receive_frame(motor->socket_fd, &rx_id, rx_data, &rx_dlc, 100) == 0) {
            if (rx_id == (COB_ID_SDO_TX_BASE + motor->node_id)) {
                if ((rx_data[0] & 0xE0) == 0x60) {
                    return 0;
                } else if ((rx_data[0] & 0xE0) == 0x80) {
                    return -1;
                }
            }
        }
    }
    
    return -1;
}

static int nmt_send(MotorDriver_t *motor, uint8_t command) {
    uint8_t data[2];
    data[0] = command;
    data[1] = motor->node_id;
    return can_send_frame(motor->socket_fd, COB_ID_NMT, data, 2);
}

/******************************************************************************
 * 公共接口实现
 ******************************************************************************/

ErrorCode_t motor_init(MotorDriver_t *motor, uint8_t node_id, const char *can_if) {
    if (motor == NULL || can_if == NULL) {
        return ERR_INVALID_PARAM;
    }

    memset(motor, 0, sizeof(MotorDriver_t));
    
    motor->node_id = node_id;
    strncpy(motor->can_interface, can_if, sizeof(motor->can_interface) - 1);
    motor->state = MOTOR_STATE_INIT;
    motor->mode = MOTOR_MODE_CSV;
    motor->max_velocity = MOTOR_MAX_VELOCITY;
    motor->max_acceleration = MOTOR_MAX_ACCELERATION;
    
    if (pthread_mutex_init(&motor->mutex, NULL) != 0) {
        return ERR_GENERAL;
    }

    motor->socket_fd = can_socket_init(can_if);
    if (motor->socket_fd < 0) {
        pthread_mutex_destroy(&motor->mutex);
        return ERR_COMM_FAIL;
    }

    motor->initialized = 1;
    motor->state = MOTOR_STATE_NOT_READY;
    
    LOG_INFO(LOG_MODULE_MOTOR, "Motor driver initialized (Node ID=%d, Interface=%s)", 
             node_id, can_if);
    
    return ERR_OK;
}

void motor_deinit(MotorDriver_t *motor) {
    if (motor == NULL || !motor->initialized) {
        return;
    }

    pthread_mutex_lock(&motor->mutex);
    
    if (motor->enabled) {
        motor_disable(motor);
    }
    
    if (motor->socket_fd >= 0) {
        close(motor->socket_fd);
        motor->socket_fd = -1;
    }
    
    motor->initialized = 0;
    motor->state = MOTOR_STATE_INIT;
    
    pthread_mutex_unlock(&motor->mutex);
    pthread_mutex_destroy(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Motor driver deinitialized");
}

ErrorCode_t motor_enable(MotorDriver_t *motor) {
    uint32_t value;
    
    if (motor == NULL || !motor->initialized) {
        return ERR_NOT_INITIALIZED;
    }

    pthread_mutex_lock(&motor->mutex);

    nmt_send(motor, 0x01);
    usleep(100000);

    if (sdo_read(motor, OD_INDEX_STATUS_WORD, 0, &value) == 0) {
        motor->status_word = (uint16_t)value;
        LOG_DEBUG(LOG_MODULE_MOTOR, "Status word: 0x%04X", motor->status_word);
    }

    if (sdo_write(motor, OD_INDEX_MODE_OF_OPERATION, 0, MOTOR_MODE_CSV, 1) < 0) {
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    usleep(10000);

    if (sdo_read(motor, OD_INDEX_MODE_DISPLAY, 0, &value) == 0) {
        motor->mode = (MotorMode_t)value;
        LOG_DEBUG(LOG_MODULE_MOTOR, "Mode display: %d", motor->mode);
    }

    sdo_write(motor, OD_INDEX_CONTROL_WORD, 0, CONTROL_WORD_SHUTDOWN, 2);
    usleep(10000);
    sdo_write(motor, OD_INDEX_CONTROL_WORD, 0, CONTROL_WORD_SWITCH_ON, 2);
    usleep(10000);
    sdo_write(motor, OD_INDEX_CONTROL_WORD, 0, CONTROL_WORD_ENABLE_OPERATION, 2);
    usleep(10000);

    if (sdo_read(motor, OD_INDEX_STATUS_WORD, 0, &value) == 0) {
        motor->status_word = (uint16_t)value;
        if ((value & 0x006F) == 0x0027) {
            motor->state = MOTOR_STATE_ENABLED;
            motor->enabled = 1;
            LOG_INFO(LOG_MODULE_MOTOR, "Motor enabled successfully");
        } else {
            motor->state = MOTOR_STATE_FAULT;
            LOG_ERROR(LOG_MODULE_MOTOR, "Motor enable failed, status: 0x%04X", value);
            pthread_mutex_unlock(&motor->mutex);
            return ERR_GENERAL;
        }
    }

    pthread_mutex_unlock(&motor->mutex);
    return ERR_OK;
}

ErrorCode_t motor_disable(MotorDriver_t *motor) {
    if (motor == NULL || !motor->initialized) {
        return ERR_NOT_INITIALIZED;
    }

    pthread_mutex_lock(&motor->mutex);

    sdo_write(motor, OD_INDEX_CONTROL_WORD, 0, CONTROL_WORD_DISABLE_VOLTAGE, 2);
    usleep(10000);
    
    motor->enabled = 0;
    motor->state = MOTOR_STATE_READY;

    pthread_mutex_unlock(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Motor disabled");
    return ERR_OK;
}

ErrorCode_t motor_set_mode(MotorDriver_t *motor, MotorMode_t mode) {
    if (motor == NULL || !motor->initialized) {
        return ERR_NOT_INITIALIZED;
    }

    pthread_mutex_lock(&motor->mutex);

    if (sdo_write(motor, OD_INDEX_MODE_OF_OPERATION, 0, mode, 1) < 0) {
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }

    motor->mode = mode;
    pthread_mutex_unlock(&motor->mutex);
    
    return ERR_OK;
}

ErrorCode_t motor_set_velocity(MotorDriver_t *motor, int32_t velocity) {
    if (motor == NULL || !motor->initialized) {
        return ERR_NOT_INITIALIZED;
    }

    pthread_mutex_lock(&motor->mutex);

    if (sdo_write(motor, OD_INDEX_TARGET_VELOCITY, 0, (uint32_t)velocity, 4) < 0) {
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }

    motor->target_velocity = velocity;
    pthread_mutex_unlock(&motor->mutex);
    
    return ERR_OK;
}

ErrorCode_t motor_set_position(MotorDriver_t *motor, int32_t position) {
    if (motor == NULL || !motor->initialized) {
        return ERR_NOT_INITIALIZED;
    }

    pthread_mutex_lock(&motor->mutex);

    if (sdo_write(motor, OD_INDEX_TARGET_POSITION, 0, (uint32_t)position, 4) < 0) {
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }

    motor->target_position = position;
    pthread_mutex_unlock(&motor->mutex);
    
    return ERR_OK;
}

ErrorCode_t motor_get_velocity(MotorDriver_t *motor, int32_t *velocity) {
    uint32_t value;
    
    if (motor == NULL || !motor->initialized || velocity == NULL) {
        return ERR_INVALID_PARAM;
    }

    pthread_mutex_lock(&motor->mutex);

    if (sdo_read(motor, OD_INDEX_VELOCITY_ACTUAL, 0, &value) < 0) {
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }

    motor->actual_velocity = (int32_t)value;
    *velocity = motor->actual_velocity;
    
    pthread_mutex_unlock(&motor->mutex);
    
    return ERR_OK;
}

ErrorCode_t motor_get_position(MotorDriver_t *motor, int32_t *position) {
    uint32_t value;
    
    if (motor == NULL || !motor->initialized || position == NULL) {
        return ERR_INVALID_PARAM;
    }

    pthread_mutex_lock(&motor->mutex);

    if (sdo_read(motor, OD_INDEX_POSITION_ACTUAL, 0, &value) < 0) {
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }

    motor->actual_position = (int32_t)value;
    *position = motor->actual_position;
    
    pthread_mutex_unlock(&motor->mutex);
    
    return ERR_OK;
}

ErrorCode_t motor_update_state(MotorDriver_t *motor) {
    uint32_t value;
    
    if (motor == NULL || !motor->initialized) {
        return ERR_NOT_INITIALIZED;
    }

    pthread_mutex_lock(&motor->mutex);

    if (sdo_read(motor, OD_INDEX_STATUS_WORD, 0, &value) == 0) {
        motor->status_word = (uint16_t)value;
    }

    pthread_mutex_unlock(&motor->mutex);
    
    return ERR_OK;
}

ErrorCode_t motor_clear_fault(MotorDriver_t *motor) {
    if (motor == NULL || !motor->initialized) {
        return ERR_NOT_INITIALIZED;
    }

    pthread_mutex_lock(&motor->mutex);

    sdo_write(motor, OD_INDEX_CONTROL_WORD, 0, 0x0080, 2);
    usleep(10000);

    pthread_mutex_unlock(&motor->mutex);
    
    return ERR_OK;
}

const char* motor_get_state_string(MotorState_t state) {
    switch (state) {
        case MOTOR_STATE_INIT:          return "INIT";
        case MOTOR_STATE_NOT_READY:     return "NOT_READY";
        case MOTOR_STATE_READY:         return "READY";
        case MOTOR_STATE_ENABLED:       return "ENABLED";
        case MOTOR_STATE_FAULT:         return "FAULT";
        case MOTOR_STATE_UNKNOWN:       return "UNKNOWN";
        default:                        return "INVALID";
    }
}

const char* motor_get_mode_string(MotorMode_t mode) {
    switch (mode) {
        case MOTOR_MODE_PP:     return "PP";
        case MOTOR_MODE_VM:     return "VM";
        case MOTOR_MODE_PV:     return "PV";
        case MOTOR_MODE_PT:     return "PT";
        case MOTOR_MODE_HM:     return "HM";
        case MOTOR_MODE_IP:     return "IP";
        case MOTOR_MODE_CSP:    return "CSP";
        case MOTOR_MODE_CSV:    return "CSV";
        case MOTOR_MODE_CST:    return "CST";
        default:                return "UNKNOWN";
    }
}
