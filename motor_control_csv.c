/*
 * Nimotion Motor CSV Control via SocketCAN
 * 
 * This program controls Nimotion CANopen motor in CSV (Cyclic Synchronous Velocity) mode
 * using Linux SocketCAN directly.
 * 
 * Compile: gcc -o motor_control_csv motor_control_csv.c
 * Run: ./motor_control_csv
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_INTERFACE "can0"
#define MOTOR_NODE_ID 1  // 电机CAN节点ID，根据实际情况修改

// CANopen COB-ID定义
#define COB_ID_NMT          0x000
#define COB_ID_SYNC         0x080
#define COB_ID_EMCY_BASE    0x080
#define COB_ID_TPDO1_BASE   0x180
#define COB_ID_RPDO1_BASE   0x200
#define COB_ID_TPDO2_BASE   0x280
#define COB_ID_RPDO2_BASE   0x300
#define COB_ID_TPDO3_BASE   0x380
#define COB_ID_RPDO3_BASE   0x400
#define COB_ID_TPDO4_BASE   0x480
#define COB_ID_RPDO4_BASE   0x500
#define COB_ID_SDO_TX_BASE  0x580
#define COB_ID_SDO_RX_BASE  0x600
#define COB_ID_NMT_ERROR    0x700

// CiA 402控制字
#define CONTROL_WORD_SHUTDOWN           0x0006
#define CONTROL_WORD_SWITCH_ON          0x0007
#define CONTROL_WORD_ENABLE_OPERATION   0x000F
#define CONTROL_WORD_DISABLE_VOLTAGE    0x0000
#define CONTROL_WORD_QUICK_STOP         0x0002
#define CONTROL_WORD_FAULT_RESET        0x0080

// CiA 402状态字掩码
#define STATUS_WORD_NOT_READY           0x0000
#define STATUS_WORD_SWITCH_ON_DISABLED  0x0040
#define STATUS_WORD_READY_TO_SWITCH_ON  0x0021
#define STATUS_WORD_SWITCHED_ON         0x0023
#define STATUS_WORD_OPERATION_ENABLED   0x0027
#define STATUS_WORD_FAULT               0x0008

// CiA 402工作模式
#define MODE_OF_OPERATION_PP    1   // 轮廓位置模式
#define MODE_OF_OPERATION_VM    2   // 速度模式
#define MODE_OF_OPERATION_PV    3   // 轮廓速度模式
#define MODE_OF_OPERATION_PT    4   // 轮廓转矩模式
#define MODE_OF_OPERATION_HM    6   // 原点回归模式
#define MODE_OF_OPERATION_IP    7   // 位置插补模式
#define MODE_OF_OPERATION_CSP   8   // 循环同步位置模式
#define MODE_OF_OPERATION_CSV   9   // 循环同步速度模式
#define MODE_OF_OPERATION_CST   10  // 循环同步转矩模式

// SDO对象字典索引
#define OD_INDEX_CONTROL_WORD       0x6040
#define OD_INDEX_STATUS_WORD        0x6041
#define OD_INDEX_MODE_OF_OPERATION  0x6060
#define OD_INDEX_MODE_DISPLAY       0x6061
#define OD_INDEX_POSITION_ACTUAL    0x6064
#define OD_INDEX_VELOCITY_ACTUAL    0x606C
#define OD_INDEX_TARGET_VELOCITY    0x60FF
#define OD_INDEX_TARGET_POSITION    0x607A

static int sock = -1;

// 初始化SocketCAN
int can_init(void) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 创建CAN套接字
    if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket creation failed");
        return -1;
    }

    // 获取接口索引
    strcpy(ifr.ifr_name, CAN_INTERFACE);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl failed");
        close(sock);
        return -1;
    }

    // 绑定套接字
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        close(sock);
        return -1;
    }

    printf("SocketCAN initialized on %s\n", CAN_INTERFACE);
    return 0;
}

// 发送CAN帧
int can_send(uint32_t can_id, uint8_t *data, uint8_t dlc) {
    struct can_frame frame;
    
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write failed");
        return -1;
    }
    
    return 0;
}

// 接收CAN帧（带超时）
int can_receive(uint32_t *can_id, uint8_t *data, uint8_t *dlc, int timeout_ms) {
    struct can_frame frame;
    fd_set rdfs;
    struct timeval tv;
    int ret;

    FD_ZERO(&rdfs);
    FD_SET(sock, &rdfs);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(sock + 1, &rdfs, NULL, NULL, &tv);
    if (ret < 0) {
        perror("Select failed");
        return -1;
    } else if (ret == 0) {
        return -1; // 超时
    }

    if (read(sock, &frame, sizeof(struct can_frame)) < 0) {
        perror("Read failed");
        return -1;
    }

    *can_id = frame.can_id;
    *dlc = frame.can_dlc;
    memcpy(data, frame.data, frame.can_dlc);
    
    return 0;
}

// 发送SDO请求（读取对象字典）
int sdo_read(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t *value) {
    uint8_t data[8];
    uint32_t rx_id;
    uint8_t rx_dlc;
    uint8_t rx_data[8];
    
    // 发送SDO下载请求（ initiate upload ）
    memset(data, 0, 8);
    data[0] = 0x40;  // CCS = 1 (initiate upload request)
    data[1] = index & 0xFF;
    data[2] = (index >> 8) & 0xFF;
    data[3] = subindex;
    
    if (can_send(COB_ID_SDO_RX_BASE + node_id, data, 8) < 0) {
        return -1;
    }
    
    // 等待SDO响应
    for (int i = 0; i < 100; i++) {
        if (can_receive(&rx_id, rx_data, &rx_dlc, 100) == 0) {
            if (rx_id == (COB_ID_SDO_TX_BASE + node_id)) {
                // 检查响应是否成功
                if ((rx_data[0] & 0xE0) == 0x80) {
                    printf("SDO abort response\n");
                    return -1;
                }
                // 提取数据（ expedited transfer ）
                if (rx_data[0] & 0x02) {  // e-bit set
                    int n = (rx_data[0] >> 2) & 0x03;  // n = number of bytes without data
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

// 发送SDO请求（写入对象字典）
int sdo_write(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size) {
    uint8_t data[8];
    uint32_t rx_id;
    uint8_t rx_dlc;
    uint8_t rx_data[8];
    
    // 发送SDO上传请求（ initiate download ）
    memset(data, 0, 8);
    data[0] = 0x23 | ((4 - size) << 2);  // CCS = 1, e=1, s=1
    data[1] = index & 0xFF;
    data[2] = (index >> 8) & 0xFF;
    data[3] = subindex;
    data[4] = value & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = (value >> 16) & 0xFF;
    data[7] = (value >> 24) & 0xFF;
    
    if (can_send(COB_ID_SDO_RX_BASE + node_id, data, 8) < 0) {
        return -1;
    }
    
    // 等待SDO响应
    for (int i = 0; i < 100; i++) {
        if (can_receive(&rx_id, rx_data, &rx_dlc, 100) == 0) {
            if (rx_id == (COB_ID_SDO_TX_BASE + node_id)) {
                // 检查响应是否成功
                if ((rx_data[0] & 0xE0) == 0x60) {  // SCS = 3 (initiate download response)
                    return 0;
                } else if ((rx_data[0] & 0xE0) == 0x80) {
                    printf("SDO abort response\n");
                    return -1;
                }
            }
        }
    }
    
    return -1;
}

// 发送NMT命令
int nmt_send(uint8_t command, uint8_t node_id) {
    uint8_t data[2];
    data[0] = command;
    data[1] = node_id;
    return can_send(COB_ID_NMT, data, 2);
}

// 电机控制函数
int motor_init(uint8_t node_id) {
    uint32_t value;
    
    printf("Initializing motor %d...\n", node_id);
    
    // 1. 发送NMT启动命令
    nmt_send(0x01, node_id);  // Start remote node
    usleep(100000);
    
    // 2. 读取状态字
    if (sdo_read(node_id, OD_INDEX_STATUS_WORD, 0, &value) == 0) {
        printf("Status word: 0x%04X\n", value);
    }
    
    // 3. 设置工作模式为CSV
    printf("Setting CSV mode...\n");
    if (sdo_write(node_id, OD_INDEX_MODE_OF_OPERATION, 0, MODE_OF_OPERATION_CSV, 1) < 0) {
        printf("Failed to set mode\n");
        return -1;
    }
    usleep(10000);
    
    // 4. 验证模式设置
    if (sdo_read(node_id, OD_INDEX_MODE_DISPLAY, 0, &value) == 0) {
        printf("Mode display: %d (CSV=%d)\n", value, MODE_OF_OPERATION_CSV);
        if (value != MODE_OF_OPERATION_CSV) {
            printf("Mode setting failed\n");
            return -1;
        }
    }
    
    // 5. 电机使能流程
    printf("Enabling motor...\n");
    
    // Shutdown
    sdo_write(node_id, OD_INDEX_CONTROL_WORD, 0, CONTROL_WORD_SHUTDOWN, 2);
    usleep(10000);
    
    // Switch on
    sdo_write(node_id, OD_INDEX_CONTROL_WORD, 0, CONTROL_WORD_SWITCH_ON, 2);
    usleep(10000);
    
    // Enable operation
    sdo_write(node_id, OD_INDEX_CONTROL_WORD, 0, CONTROL_WORD_ENABLE_OPERATION, 2);
    usleep(10000);
    
    // 6. 检查状态
    if (sdo_read(node_id, OD_INDEX_STATUS_WORD, 0, &value) == 0) {
        printf("Status word after enable: 0x%04X\n", value);
        if ((value & 0x006F) == STATUS_WORD_OPERATION_ENABLED) {
            printf("Motor enabled successfully!\n");
        } else {
            printf("Motor enable failed, status: 0x%04X\n", value);
            return -1;
        }
    }
    
    return 0;
}

// 设置目标速度
int motor_set_velocity(uint8_t node_id, int32_t velocity) {
    return sdo_write(node_id, OD_INDEX_TARGET_VELOCITY, 0, (uint32_t)velocity, 4);
}

// 读取实际速度
int motor_get_velocity(uint8_t node_id, int32_t *velocity) {
    uint32_t value;
    if (sdo_read(node_id, OD_INDEX_VELOCITY_ACTUAL, 0, &value) == 0) {
        *velocity = (int32_t)value;
        return 0;
    }
    return -1;
}

// 读取实际位置
int motor_get_position(uint8_t node_id, int32_t *position) {
    uint32_t value;
    if (sdo_read(node_id, OD_INDEX_POSITION_ACTUAL, 0, &value) == 0) {
        *position = (int32_t)value;
        return 0;
    }
    return -1;
}

// 电机失能
int motor_disable(uint8_t node_id) {
    printf("Disabling motor...\n");
    sdo_write(node_id, OD_INDEX_CONTROL_WORD, 0, CONTROL_WORD_DISABLE_VOLTAGE, 2);
    usleep(10000);
    return 0;
}

// 主函数
int main(int argc, char *argv[]) {
    uint8_t node_id = MOTOR_NODE_ID;
    
    if (argc > 1) {
        node_id = atoi(argv[1]);
    }
    
    printf("========================================\n");
    printf("Nimotion Motor CSV Control via SocketCAN\n");
    printf("Node ID: %d\n", node_id);
    printf("Interface: %s\n", CAN_INTERFACE);
    printf("========================================\n\n");
    
    // 初始化CAN
    if (can_init() < 0) {
        printf("Failed to initialize CAN\n");
        return 1;
    }
    
    // 初始化电机
    if (motor_init(node_id) < 0) {
        printf("Motor initialization failed\n");
        close(sock);
        return 1;
    }
    
    printf("\n========================================\n");
    printf("Running CSV mode - Sine wave velocity\n");
    printf("Press Ctrl+C to stop\n");
    printf("========================================\n\n");
    
    // CSV模式运行 - 正弦波速度曲线
    double t = 0;
    int32_t velocity, actual_vel, actual_pos;
    
    while (1) {
        // 计算目标速度（正弦波）
        velocity = (int32_t)(5000.0 * sin(2.0 * M_PI * 0.5 * t));  // 5000 units/s, 0.5Hz
        
        // 发送目标速度
        if (motor_set_velocity(node_id, velocity) < 0) {
            printf("Failed to set velocity\n");
            break;
        }
        
        // 读取实际速度和位置
        if (motor_get_velocity(node_id, &actual_vel) == 0 &&
            motor_get_position(node_id, &actual_pos) == 0) {
            printf("t=%.2fs, Target=%6d, ActualVel=%6d, ActualPos=%10d\r", 
                   t, velocity, actual_vel, actual_pos);
            fflush(stdout);
        }
        
        // 10ms周期
        usleep(10000);
        t += 0.01;
        
        // 运行10秒后停止
        if (t > 10.0) {
            break;
        }
    }
    
    printf("\n\nStopping motor...\n");
    
    // 停止电机
    motor_set_velocity(node_id, 0);
    usleep(100000);
    motor_disable(node_id);
    
    // 发送NMT停止命令
    nmt_send(0x02, node_id);  // Stop remote node
    
    printf("Motor stopped.\n");
    
    close(sock);
    return 0;
}
