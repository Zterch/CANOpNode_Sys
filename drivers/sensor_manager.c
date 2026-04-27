/******************************************************************************
 * @file    sensor_manager.c
 * @brief   统一传感器管理器实现 - RS485传感器轮询管理
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#define _GNU_SOURCE
#include "sensor_manager.h"
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>

/* 外部函数声明 - 来自 rs485_bus.c */
extern int serial_open(const char *device, int baudrate);
extern int serial_configure(int fd, int baudrate, int data_bits, int stop_bits, char parity);

/* 定义日志模块 */
#define LOG_MODULE_SENSOR  LOG_MODULE_ENCODER  /* 复用编码器模块日志 */

/* 定义 M_PI (如果未定义) */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 定义错误码 */
#define ERR_DEV_NOT_FOUND  -3

/******************************************************************************
 * 私有变量
 ******************************************************************************/

/* 编码器绳子参数 */
static float s_rope_drum_diameter = 100.0f;     /* 卷筒直径(mm) */
static float s_rope_length_per_turn = 314.16f;  /* 每圈绳长(mm) */
static float s_rope_length_base = 0.0f;         /* 基准长度(mm) */
static uint32_t s_encoder_resolution = 4096;    /* 编码器分辨率 */
static uint32_t s_encoder_zero_offset = 0;      /* 零点偏移 */

/* 压力传感器参数 */
static int16_t s_pressure_zero_offset = 0;      /* 压力传感器去皮偏移 */

/* 数据文件路径 */
#define ENCODER_DATA_FILE   "share/encoder_rope_data.txt"
#define PRESSURE_DATA_FILE  "share/pressure_zero.txt"

/******************************************************************************
 * 辅助函数
 ******************************************************************************/

/* 获取当前时间(微秒) */
static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/******************************************************************************
 * 数据文件加载/保存
 ******************************************************************************/

/* 加载编码器绳长数据 */
static int load_encoder_data(void) {
    FILE *fp = fopen(ENCODER_DATA_FILE, "r");
    if (fp == NULL) {
        printf("[SENSOR] No encoder data file found, using defaults\n");
        return -1;
    }
    
    char line[128];
    while (fgets(line, sizeof(line), fp)) {
        /* 跳过注释行 */
        if (line[0] == '#') continue;
        
        /* 解析 BASE_LENGTH_MM */
        if (strncmp(line, "BASE_LENGTH_MM=", 15) == 0) {
            sscanf(line + 15, "%f", &s_rope_length_base);
        }
        /* 解析 ZERO_OFFSET */
        else if (strncmp(line, "ZERO_OFFSET=", 12) == 0) {
            sscanf(line + 12, "%u", &s_encoder_zero_offset);
        }
        /* 解析 DRUM_DIAMETER */
        else if (strncmp(line, "DRUM_DIAMETER=", 14) == 0) {
            sscanf(line + 14, "%f", &s_rope_drum_diameter);
        }
        /* 解析 ENCODER_RESOLUTION */
        else if (strncmp(line, "ENCODER_RESOLUTION=", 19) == 0) {
            sscanf(line + 19, "%u", &s_encoder_resolution);
        }
    }
    
    fclose(fp);
    
    /* 重新计算每圈绳长 */
    s_rope_length_per_turn = M_PI * s_rope_drum_diameter;
    
    printf("[SENSOR] Loaded encoder data: base=%.2fmm, offset=%u, drum=%.2fmm, res=%u\n",
           s_rope_length_base, s_encoder_zero_offset, s_rope_drum_diameter, s_encoder_resolution);
    
    return 0;
}

/* 保存编码器绳长数据 */
static int save_encoder_data(void) {
    FILE *fp = fopen(ENCODER_DATA_FILE, "w");
    if (fp == NULL) {
        printf("[SENSOR] Failed to save encoder data\n");
        return -1;
    }
    
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char time_str[32];
    strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_info);
    
    fprintf(fp, "# Encoder Rope Length Data\n");
    fprintf(fp, "# Saved at: %s\n", time_str);
    fprintf(fp, "BASE_LENGTH_MM=%.4f\n", s_rope_length_base);
    fprintf(fp, "ZERO_OFFSET=%u\n", s_encoder_zero_offset);
    fprintf(fp, "DRUM_DIAMETER=%.4f\n", s_rope_drum_diameter);
    fprintf(fp, "ENCODER_RESOLUTION=%u\n", s_encoder_resolution);
    
    fclose(fp);
    
    printf("[SENSOR] Saved encoder data: base=%.2fmm, offset=%u\n",
           s_rope_length_base, s_encoder_zero_offset);
    
    return 0;
}

/* 加载压力传感器去皮数据 */
static int load_pressure_data(void) {
    FILE *fp = fopen(PRESSURE_DATA_FILE, "r");
    if (fp == NULL) {
        printf("[SENSOR] No pressure data file found, using zero offset\n");
        return -1;
    }
    
    char line[128];
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#') continue;
        
        if (strncmp(line, "ZERO_OFFSET=", 12) == 0) {
            int temp;
            sscanf(line + 12, "%d", &temp);
            s_pressure_zero_offset = (int16_t)temp;
        }
    }
    
    fclose(fp);
    
    printf("[SENSOR] Loaded pressure zero offset: %d\n", s_pressure_zero_offset);
    
    return 0;
}

/* 保存压力传感器去皮数据 */
static int save_pressure_data(void) {
    FILE *fp = fopen(PRESSURE_DATA_FILE, "w");
    if (fp == NULL) {
        printf("[SENSOR] Failed to save pressure data\n");
        return -1;
    }
    
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char time_str[32];
    strftime(time_str, sizeof(time_str), "%H:%M:%S", tm_info);
    
    fprintf(fp, "# Pressure Sensor Zero Data\n");
    fprintf(fp, "# Saved at: %s\n", time_str);
    fprintf(fp, "ZERO_OFFSET=%d\n", s_pressure_zero_offset);
    
    fclose(fp);
    
    printf("[SENSOR] Saved pressure zero offset: %d\n", s_pressure_zero_offset);
    
    return 0;
}

/* 计算Modbus CRC16 */
static uint16_t calc_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* 清空串口接收缓冲区 */
static void flush_rx_buffer(int fd) {
    uint8_t tmp[256];
    while (read(fd, tmp, sizeof(tmp)) > 0) {
        /* 持续读取直到缓冲区为空 */
    }
}

/******************************************************************************
 * Modbus RTU 通信函数
 ******************************************************************************/

/* 发送Modbus请求并接收响应 */
static ErrorCode_t modbus_read_registers(SensorManager_t *manager,
                                          uint8_t slave_addr,
                                          uint8_t func_code,
                                          uint16_t reg_addr,
                                          uint16_t reg_count,
                                          uint8_t *rx_buf,
                                          int *rx_len,
                                          uint32_t timeout_ms) {
    uint8_t tx_buf[8];
    
    /* 构建请求帧 */
    tx_buf[0] = slave_addr;
    tx_buf[1] = func_code;
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (reg_count >> 8) & 0xFF;
    tx_buf[5] = reg_count & 0xFF;
    
    /* 计算CRC */
    uint16_t crc = calc_crc16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    
    /* 清空接收缓冲区 */
    flush_rx_buffer(manager->rs485_fd);
    
    /* 帧间隔 (3.5字符时间 @115200bps ≈ 0.3ms, 取1ms安全) */
    usleep(1000);
    
    /* 发送请求 */
    if (write(manager->rs485_fd, tx_buf, 8) != 8) {
        return ERR_COMM_FAIL;
    }
    tcflush(manager->rs485_fd, TCOFLUSH);
    
    /* 等待响应 */
    usleep(2000);  /* 最小等待时间 */
    
    /* 接收数据 */
    *rx_len = 0;
    uint32_t elapsed_ms = 0;
    
    while (elapsed_ms < timeout_ms) {
        int n = read(manager->rs485_fd, rx_buf + *rx_len, 256 - *rx_len);
        if (n > 0) {
            *rx_len += n;
            
            /* 检查是否接收完整 */
            if (*rx_len >= 5) {
                uint8_t expected_len;
                if (func_code == 0x03 || func_code == 0x04) {
                    expected_len = 5 + rx_buf[2];  /* 地址+功能码+字节数+数据+CRC */
                } else {
                    expected_len = 5;  /* 异常响应 */
                }
                
                if (*rx_len >= expected_len) {
                    break;
                }
            }
        }
        
        usleep(1000);
        elapsed_ms++;
    }
    
    if (*rx_len < 5) {
        return ERR_TIMEOUT;
    }
    
    /* 验证响应 */
    if (rx_buf[0] != slave_addr) {
        return ERR_COMM_FAIL;
    }
    
    if (rx_buf[1] != func_code) {
        return ERR_COMM_FAIL;
    }
    
    /* 验证CRC */
    int data_len = *rx_len - 2;
    uint16_t rx_crc = (rx_buf[*rx_len - 1] << 8) | rx_buf[*rx_len - 2];
    uint16_t calc_crc = calc_crc16(rx_buf, data_len);
    
    if (rx_crc != calc_crc) {
        return ERR_COMM_FAIL;
    }
    
    return ERR_OK;
}

/******************************************************************************
 * 传感器数据解析
 ******************************************************************************/

/* 读取并解析编码器数据 */
static ErrorCode_t read_encoder(SensorManager_t *manager, SensorData_t *data) {
    uint8_t rx_buf[16];
    int rx_len;
    
    ErrorCode_t ret = modbus_read_registers(manager,
                                             manager->configs[SENSOR_TYPE_ENCODER].slave_addr,
                                             manager->configs[SENSOR_TYPE_ENCODER].func_code,
                                             manager->configs[SENSOR_TYPE_ENCODER].reg_addr,
                                             manager->configs[SENSOR_TYPE_ENCODER].reg_count,
                                             rx_buf, &rx_len, 50);
    
    if (ret != ERR_OK) {
        return ret;
    }
    
    /* 解析32位多圈值 (大端模式) */
    if (rx_buf[2] == 4) {
        uint32_t multi_turn = ((uint32_t)rx_buf[3] << 24) |
                              ((uint32_t)rx_buf[4] << 16) |
                              ((uint32_t)rx_buf[5] << 8) |
                              ((uint32_t)rx_buf[6]);
        
        data->data.encoder.multi_turn_value = multi_turn;
        
        /* 计算角度 */
        data->data.encoder.angle_deg = (float)multi_turn * 360.0f / (float)s_encoder_resolution;
        
        /* 计算绳子长度 */
        int64_t relative_pulses = (int64_t)multi_turn - (int64_t)s_encoder_zero_offset;
        float relative_turns = (float)relative_pulses / (float)s_encoder_resolution;
        data->data.encoder.rope_length_mm = s_rope_length_base + 
                                            (relative_turns * s_rope_length_per_turn);
        
        data->data_valid = 1;
        data->last_read_us = get_time_us();
        
        return ERR_OK;
    }
    
    return ERR_COMM_FAIL;
}

/* 读取并解析压力传感器数据 */
static ErrorCode_t read_pressure(SensorManager_t *manager, SensorData_t *data) {
    uint8_t rx_buf[16];
    int rx_len;
    
    ErrorCode_t ret = modbus_read_registers(manager,
                                             manager->configs[SENSOR_TYPE_PRESSURE].slave_addr,
                                             manager->configs[SENSOR_TYPE_PRESSURE].func_code,
                                             manager->configs[SENSOR_TYPE_PRESSURE].reg_addr,
                                             manager->configs[SENSOR_TYPE_PRESSURE].reg_count,
                                             rx_buf, &rx_len, 50);
    
    if (ret != ERR_OK) {
        return ret;
    }
    
    /* 解析16位压力值 (大端模式) - 数据为有符号整型 */
    if (rx_buf[2] == 2) {
        /* 先读取为无符号，再转换为有符号 */
        uint16_t raw_u16 = ((uint16_t)rx_buf[3] << 8) | (uint16_t)rx_buf[4];
        int16_t raw_s16 = (int16_t)raw_u16;  /* 关键：转换为有符号 */
        
        data->data.pressure.raw_value = raw_s16;
        data->data.pressure.zero_offset = s_pressure_zero_offset;
        
        /* 应用去皮并转换为kg (3位小数) */
        int16_t adjusted = raw_s16 - s_pressure_zero_offset;
        data->data.pressure.pressure_kg = (float)adjusted / 10.0f;
        
        data->data_valid = 1;
        data->last_read_us = get_time_us();
        
        return ERR_OK;
    }
    
    return ERR_COMM_FAIL;
}

/******************************************************************************
 * 管理线程
 ******************************************************************************/

static void* sensor_manager_thread(void* arg) {
    SensorManager_t *manager = (SensorManager_t*)arg;
    
    LOG_INFO(LOG_MODULE_SENSOR, "Sensor manager thread started (50Hz)");
    
    manager->running = 1;
    
    while (manager->running) {
        uint64_t cycle_start = get_time_us();
        manager->cycle_count++;
        
        /* === 时间片1: 读取编码器 (0-6ms) === */
        pthread_mutex_lock(&manager->bus_mutex);
        
        if (read_encoder(manager, &manager->datas[SENSOR_TYPE_ENCODER]) == ERR_OK) {
            manager->datas[SENSOR_TYPE_ENCODER].read_count++;
        } else {
            manager->datas[SENSOR_TYPE_ENCODER].error_count++;
            manager->total_errors++;
        }
        
        pthread_mutex_unlock(&manager->bus_mutex);
        
        /* === 时间片2: 读取压力传感器 (6-12ms) === */
        pthread_mutex_lock(&manager->bus_mutex);
        
        if (read_pressure(manager, &manager->datas[SENSOR_TYPE_PRESSURE]) == ERR_OK) {
            manager->datas[SENSOR_TYPE_PRESSURE].read_count++;
        } else {
            manager->datas[SENSOR_TYPE_PRESSURE].error_count++;
            manager->total_errors++;
        }
        
        pthread_mutex_unlock(&manager->bus_mutex);
        
        /* === 等待完成20ms周期 === */
        uint64_t cycle_elapsed = get_time_us() - cycle_start;
        int64_t sleep_us = 20000 - (int64_t)cycle_elapsed;  /* 20ms = 20000us */
        
        if (sleep_us > 0) {
            usleep((useconds_t)sleep_us);
        }
    }
    
    LOG_INFO(LOG_MODULE_SENSOR, "Sensor manager thread stopped");
    
    return NULL;
}

/******************************************************************************
 * 公共接口实现
 ******************************************************************************/

ErrorCode_t sensor_mgr_init(SensorManager_t *manager, const char *device, int baudrate) {
    if (manager == NULL || device == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(manager, 0, sizeof(SensorManager_t));
    
    /* 初始化编码器配置 */
    manager->configs[SENSOR_TYPE_ENCODER].type = SENSOR_TYPE_ENCODER;
    manager->configs[SENSOR_TYPE_ENCODER].slave_addr = 2;       /* 编码器地址 */
    manager->configs[SENSOR_TYPE_ENCODER].reg_addr = 0x0000;    /* 多圈值寄存器 */
    manager->configs[SENSOR_TYPE_ENCODER].reg_count = 2;        /* 2个寄存器 = 4字节 */
    manager->configs[SENSOR_TYPE_ENCODER].func_code = 0x03;
    manager->configs[SENSOR_TYPE_ENCODER].read_timeout_ms = 50;
    manager->configs[SENSOR_TYPE_ENCODER].name = "Encoder";
    
    /* 初始化压力传感器配置 */
    manager->configs[SENSOR_TYPE_PRESSURE].type = SENSOR_TYPE_PRESSURE;
    manager->configs[SENSOR_TYPE_PRESSURE].slave_addr = 1;      /* 压力传感器地址 */
    manager->configs[SENSOR_TYPE_PRESSURE].reg_addr = 0x0000;   /* 压力值寄存器 */
    manager->configs[SENSOR_TYPE_PRESSURE].reg_count = 1;       /* 1个寄存器 = 2字节 */
    manager->configs[SENSOR_TYPE_PRESSURE].func_code = 0x03;
    manager->configs[SENSOR_TYPE_PRESSURE].read_timeout_ms = 50;
    manager->configs[SENSOR_TYPE_PRESSURE].name = "Pressure";
    
    /* 初始化互斥锁 */
    if (pthread_mutex_init(&manager->bus_mutex, NULL) != 0) {
        return ERR_GENERAL;
    }
    
    /* 打开RS485串口 */
    manager->rs485_fd = serial_open(device, baudrate);
    if (manager->rs485_fd < 0) {
        pthread_mutex_destroy(&manager->bus_mutex);
        return ERR_DEV_NOT_FOUND;
    }
    
    /* 配置串口参数 */
    if (serial_configure(manager->rs485_fd, baudrate, 8, 1, 'N') != 0) {
        close(manager->rs485_fd);
        pthread_mutex_destroy(&manager->bus_mutex);
        return ERR_GENERAL;
    }
    
    manager->initialized = 1;
    
    /* 加载保存的数据 */
    load_encoder_data();
    load_pressure_data();
    
    LOG_INFO(LOG_MODULE_SENSOR, "Sensor manager initialized");
    LOG_INFO(LOG_MODULE_SENSOR, "  Encoder: addr=%d, reg=0x%04X",
             manager->configs[SENSOR_TYPE_ENCODER].slave_addr,
             manager->configs[SENSOR_TYPE_ENCODER].reg_addr);
    LOG_INFO(LOG_MODULE_SENSOR, "  Pressure: addr=%d, reg=0x%04X",
             manager->configs[SENSOR_TYPE_PRESSURE].slave_addr,
             manager->configs[SENSOR_TYPE_PRESSURE].reg_addr);
    
    return ERR_OK;
}

void sensor_mgr_deinit(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return;
    }
    
    /* 停止线程 */
    sensor_mgr_stop(manager);
    
    /* 更新基准值 - 实现掉电记忆功能 */
    SensorData_t *encoder_data = &manager->datas[SENSOR_TYPE_ENCODER];
    if (encoder_data->data_valid) {
        /* 保存当前的绳长作为新的基准长度 */
        s_rope_length_base = encoder_data->data.encoder.rope_length_mm;
        /* 保存当前的编码器读数作为新的零点偏移 */
        s_encoder_zero_offset = encoder_data->data.encoder.multi_turn_value;
        
        printf("[SENSOR] Updating encoder baseline before save: base=%.2fmm, offset=%u\n",
               s_rope_length_base, s_encoder_zero_offset);
    }
    
    /* 保存数据 */
    save_encoder_data();
    save_pressure_data();
    
    /* 关闭串口 */
    if (manager->rs485_fd >= 0) {
        close(manager->rs485_fd);
        manager->rs485_fd = -1;
    }
    
    /* 销毁互斥锁 */
    pthread_mutex_destroy(&manager->bus_mutex);
    
    manager->initialized = 0;
    
    LOG_INFO(LOG_MODULE_SENSOR, "Sensor manager deinitialized");
}

ErrorCode_t sensor_mgr_start(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    if (manager->running) {
        return ERR_OK;  /* 已经在运行 */
    }
    
    /* 创建管理线程 */
    if (pthread_create(&manager->manager_thread, NULL, sensor_manager_thread, manager) != 0) {
        return ERR_GENERAL;
    }
    
    LOG_INFO(LOG_MODULE_SENSOR, "Sensor manager thread started");
    
    return ERR_OK;
}

void sensor_mgr_stop(SensorManager_t *manager) {
    if (manager == NULL || !manager->running) {
        return;
    }
    
    manager->running = 0;
    
    /* 等待线程结束 */
    pthread_join(manager->manager_thread, NULL);
    
    LOG_INFO(LOG_MODULE_SENSOR, "Sensor manager thread stopped");
}

ErrorCode_t sensor_mgr_get_data(SensorManager_t *manager, SensorType_t type, SensorData_t *data) {
    if (manager == NULL || !manager->initialized || data == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    if (type >= SENSOR_TYPE_COUNT) {
        return ERR_INVALID_PARAM;
    }
    
    /* 复制数据 */
    pthread_mutex_lock(&manager->bus_mutex);
    memcpy(data, &manager->datas[type], sizeof(SensorData_t));
    pthread_mutex_unlock(&manager->bus_mutex);
    
    return ERR_OK;
}

ErrorCode_t sensor_mgr_set_encoder_rope_params(SensorManager_t *manager, 
                                                float drum_diameter, 
                                                uint32_t resolution) {
    if (manager == NULL || !manager->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    if (drum_diameter <= 0 || resolution == 0) {
        return ERR_INVALID_PARAM;
    }
    
    s_rope_drum_diameter = drum_diameter;
    s_encoder_resolution = resolution;
    s_rope_length_per_turn = M_PI * drum_diameter;
    
    LOG_INFO(LOG_MODULE_SENSOR, "Encoder rope params: drum=%.2fmm, resolution=%u, per_turn=%.2fmm",
             drum_diameter, resolution, s_rope_length_per_turn);
    
    return ERR_OK;
}

ErrorCode_t sensor_mgr_encoder_zero_calibration(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 读取当前编码器值作为零点 */
    SensorData_t data;
    
    pthread_mutex_lock(&manager->bus_mutex);
    ErrorCode_t ret = read_encoder(manager, &data);
    pthread_mutex_unlock(&manager->bus_mutex);
    
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_SENSOR, "Zero calibration failed: cannot read encoder");
        return ret;
    }
    
    s_encoder_zero_offset = data.data.encoder.multi_turn_value;
    
    LOG_INFO(LOG_MODULE_SENSOR, "Zero calibration done: offset=%u", s_encoder_zero_offset);
    
    return ERR_OK;
}

ErrorCode_t sensor_mgr_pressure_tare(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 多次读取取平均值，提高去皮精度 */
    int16_t sum = 0;
    int valid_reads = 0;
    
    for (int i = 0; i < 5; i++) {
        SensorData_t data;
        pthread_mutex_lock(&manager->bus_mutex);
        ErrorCode_t ret = read_pressure(manager, &data);
        pthread_mutex_unlock(&manager->bus_mutex);
        
        if (ret == ERR_OK && data.data_valid) {
            sum += data.data.pressure.raw_value;
            valid_reads++;
        }
        usleep(100000); /* 100ms间隔 */
    }
    
    if (valid_reads < 3) {
        LOG_ERROR(LOG_MODULE_SENSOR, "Pressure tare failed: not enough valid readings");
        return ERR_COMM_FAIL;
    }
    
    /* 计算平均原始值作为去皮偏移 */
    s_pressure_zero_offset = sum / valid_reads;
    
    LOG_INFO(LOG_MODULE_SENSOR, "Pressure tare done: offset=%d, based on %d readings", 
             s_pressure_zero_offset, valid_reads);
    
    /* 立即保存去皮值到文件 */
    save_pressure_data();
    
    return ERR_OK;
}

ErrorCode_t sensor_mgr_set_encoder_base_length(SensorManager_t *manager, float base_length_mm) {
    if (manager == NULL || !manager->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    s_rope_length_base = base_length_mm;
    save_encoder_data();  /* 立即保存到文件 */
    
    LOG_INFO(LOG_MODULE_SENSOR, "Encoder base length set to %.2fmm", base_length_mm);
    
    return ERR_OK;
}

float sensor_mgr_get_success_rate(SensorManager_t *manager, SensorType_t type) {
    if (manager == NULL || type >= SENSOR_TYPE_COUNT) {
        return 0.0f;
    }
    
    uint32_t total = manager->datas[type].read_count + manager->datas[type].error_count;
    
    if (total == 0) {
        return 0.0f;
    }
    
    return 100.0f * (float)manager->datas[type].read_count / (float)total;
}

void sensor_mgr_print_status(SensorManager_t *manager) {
    if (manager == NULL) {
        return;
    }
    
    printf("\n=== Sensor Manager Status ===\n");
    printf("Cycle count: %lu\n", (unsigned long)manager->cycle_count);
    printf("Total errors: %lu\n", (unsigned long)manager->total_errors);
    
    for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
        SensorConfig_t *cfg = &manager->configs[i];
        SensorData_t *data = &manager->datas[i];
        
        printf("\n%s (addr=%d):\n", cfg->name, cfg->slave_addr);
        printf("  Reads: %u, Errors: %u, Success: %.1f%%\n",
               data->read_count, data->error_count,
               sensor_mgr_get_success_rate(manager, i));
        
        if (data->data_valid) {
            if (i == SENSOR_TYPE_ENCODER) {
                printf("  Multi-turn: %u\n", data->data.encoder.multi_turn_value);
                printf("  Angle: %.2f deg\n", data->data.encoder.angle_deg);
                printf("  Rope: %.2f mm\n", data->data.encoder.rope_length_mm);
            } else if (i == SENSOR_TYPE_PRESSURE) {
                printf("  Pressure: %.3f kg (raw=%d, offset=%d)\n", 
                       data->data.pressure.pressure_kg,
                       data->data.pressure.raw_value,
                       data->data.pressure.zero_offset);
            }
        }
    }
    printf("\n");
}
