/******************************************************************************
 * @file    rs485_bus.c
 * @brief   RS485总线管理器 - 管理共享的RS485串口
 * @author  System Architect
 * @date    2026-04-20
 * @version 1.0.0
 * 
 * 说明：
 * - 编码器和压力传感器共用同一个USB转RS485设备
 * - 此模块提供串口的统一管理和线程安全访问
 ******************************************************************************/

#include "rs485_bus.h"
#include "../utils/logger.h"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>

static RS485Bus_t g_rs485_bus = {0};

/******************************************************************************
 * CRC16计算函数（标准Modbus位运算实现）
 ******************************************************************************/
uint16_t crc16_modbus(const uint8_t *pbuf, uint8_t num) {
    int i, j;
    uint16_t wcrc = 0xFFFF;
    
    for (i = 0; i < num; i++) {
        wcrc ^= (uint16_t)(pbuf[i]);
        for (j = 0; j < 8; j++) {
            if (wcrc & 0x0001) {
                wcrc >>= 1;
                wcrc ^= 0xA001;
            } else {
                wcrc >>= 1;
            }
        }
    }
    
    return wcrc;
}

/******************************************************************************
 * 串口操作函数
 ******************************************************************************/
static int serial_open(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        LOG_ERROR(LOG_MODULE_SYS, "Failed to open %s: %s", device, strerror(errno));
        return -1;
    }
    
    /* 清除非阻塞标志 */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0) {
        LOG_ERROR(LOG_MODULE_SYS, "tcgetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    
    /* 设置波特率 */
    speed_t baud = B9600;
    switch (baudrate) {
        case 2400:   baud = B2400;   break;
        case 4800:   baud = B4800;   break;
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        default:     baud = B9600;   break;
    }
    
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    /* 8N1 */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    
    /* 原始模式 */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    
    /* 最小读取字节数和超时 */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        LOG_ERROR(LOG_MODULE_SYS, "tcsetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    
    LOG_INFO(LOG_MODULE_SYS, "RS485 bus %s opened at %d baud", device, baudrate);
    return fd;
}

static void serial_close(int fd) {
    if (fd >= 0) {
        tcflush(fd, TCIOFLUSH);
        close(fd);
    }
}

/******************************************************************************
 * RS485总线管理接口
 ******************************************************************************/

ErrorCode_t rs485_bus_init(const char *device, int baudrate) {
    if (g_rs485_bus.initialized) {
        return ERR_ALREADY_INIT;
    }
    
    if (pthread_mutex_init(&g_rs485_bus.mutex, NULL) != 0) {
        return ERR_GENERAL;
    }
    
    g_rs485_bus.fd = serial_open(device, baudrate);
    if (g_rs485_bus.fd < 0) {
        pthread_mutex_destroy(&g_rs485_bus.mutex);
        return ERR_DEVICE_NOT_FOUND;
    }
    
    strncpy(g_rs485_bus.device, device, sizeof(g_rs485_bus.device) - 1);
    g_rs485_bus.baudrate = baudrate;
    g_rs485_bus.initialized = 1;
    
    LOG_INFO(LOG_MODULE_SYS, "RS485 bus initialized (device=%s, baud=%d)", device, baudrate);
    return ERR_OK;
}

void rs485_bus_deinit(void) {
    if (!g_rs485_bus.initialized) {
        return;
    }
    
    pthread_mutex_lock(&g_rs485_bus.mutex);
    serial_close(g_rs485_bus.fd);
    g_rs485_bus.fd = -1;
    g_rs485_bus.initialized = 0;
    pthread_mutex_unlock(&g_rs485_bus.mutex);
    
    pthread_mutex_destroy(&g_rs485_bus.mutex);
    
    LOG_INFO(LOG_MODULE_SYS, "RS485 bus deinitialized");
}

int rs485_bus_get_fd(void) {
    return g_rs485_bus.fd;
}

pthread_mutex_t* rs485_bus_get_mutex(void) {
    return &g_rs485_bus.mutex;
}

ErrorCode_t rs485_bus_send(const uint8_t *data, int len, int timeout_ms) {
    if (!g_rs485_bus.initialized || g_rs485_bus.fd < 0) {
        return ERR_NOT_INITIALIZED;
    }
    
    fd_set writefds;
    struct timeval tv;
    
    FD_ZERO(&writefds);
    FD_SET(g_rs485_bus.fd, &writefds);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(g_rs485_bus.fd + 1, NULL, &writefds, NULL, &tv);
    if (ret <= 0) {
        return ERR_TIMEOUT;
    }
    
    ret = write(g_rs485_bus.fd, data, len);
    if (ret != len) {
        return ERR_COMM_FAIL;
    }
    
    return ERR_OK;
}

ErrorCode_t rs485_bus_receive(uint8_t *data, int max_len, int *received_len, int timeout_ms) {
    if (!g_rs485_bus.initialized || g_rs485_bus.fd < 0) {
        return ERR_NOT_INITIALIZED;
    }
    
    fd_set readfds;
    struct timeval tv;
    
    FD_ZERO(&readfds);
    FD_SET(g_rs485_bus.fd, &readfds);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(g_rs485_bus.fd + 1, &readfds, NULL, NULL, &tv);
    if (ret <= 0) {
        return ERR_TIMEOUT;
    }
    
    ret = read(g_rs485_bus.fd, data, max_len);
    if (ret < 0) {
        return ERR_COMM_FAIL;
    }
    
    if (received_len) {
        *received_len = ret;
    }
    
    return ERR_OK;
}

void rs485_bus_flush(void) {
    if (g_rs485_bus.initialized && g_rs485_bus.fd >= 0) {
        tcflush(g_rs485_bus.fd, TCIOFLUSH);
    }
}
