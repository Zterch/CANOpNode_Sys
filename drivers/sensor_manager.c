#define _GNU_SOURCE
#include "sensor_manager.h"
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <limits.h>
#include "../config/system_config.h"

/* 编码器参数 */
static float s_rope_drum_diameter = 100.0f;
static float s_rope_length_per_turn = 314.16f;
static float s_rope_length_base = 0.0f;
static uint32_t s_encoder_resolution = 4096;
static uint32_t s_encoder_zero_offset = 0;

/* 编码器数据校验参数 */
static uint32_t s_last_valid_encoder = 0;
static int s_encoder_first_read = 1;  /* 首次读取标志 */
static int s_encoder_consecutive_errors = 0;
#define ENCODER_MAX_DELTA_PER_CYCLE 10000
#define ENCODER_ERROR_THRESHOLD 5

/* 压力传感器参数 */
static int16_t s_pressure_zero_offset = 0;

/* Modbus功能码 */
#define MODBUS_READ_HOLDING 0x03

/* 编码器数据文件路径 */
#define ENCODER_DATA_FILE "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/share/encoder_rope_data.txt"
#define PRESSURE_DATA_FILE "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/share/pressure_data.txt"

/* 日志模块 */
#define LOG_MODULE_SENSOR "SENSOR"

/* 获取微秒时间戳 */
static uint64_t get_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

/* 编码器最大值（数据手册规定范围 0~2147483647） */
#define ENCODER_MAX_VALUE 2147483647U
#define ENCODER_HALF_RANGE 1073741824  /* 2^30，半圈脉冲数 */
#define ENCODER_FULL_SIGNED_RANGE 2147483648  /* 2^31，有符号范围 */

/* 计算两个uint32_t编码器值的差值（正确处理回绕）
 * 编码器值在 0~2147483647 范围内循环
 * 当值超过最大值时回绕到0
 * 
 * 原理：
 * 1. 先计算有符号差值 delta = (int32_t)(current - last)
 * 2. 如果 delta > 2^30 (半圈)，说明是负方向回绕，delta -= 2^31
 * 3. 如果 delta < -2^30 (负半圈)，说明是正方向回绕，delta += 2^31
 * 
 * 这样可以把回绕导致的巨大差值映射回合理的范围
 */
static int32_t calculate_encoder_delta(uint32_t current, uint32_t last) {
    /* 计算有符号差值 */
    int32_t delta = (int32_t)(current - last);
    
    /* 处理负方向回绕：例如 current=2147483646, last=0
     * delta = 2147483646 (很大，超过半圈)
     * 实际应该是 -2 (负方向移动了2个脉冲)
     */
    if (delta > ENCODER_HALF_RANGE) {
        delta -= ENCODER_FULL_SIGNED_RANGE;  /* 2147483646 - 2147483648 = -2 */
    }
    /* 处理正方向回绕：例如 current=1, last=2147483641
     * delta = -2147483640 (很小，小于负半圈)
     * 实际应该是 +8 (正方向移动了8个脉冲)
     */
    else if (delta < -ENCODER_HALF_RANGE) {
        delta += ENCODER_FULL_SIGNED_RANGE;  /* -2147483640 + 2147483648 = +8 */
    }
    
    return delta;
}

/* Modbus读取保持寄存器 */
static ErrorCode_t modbus_read_registers(SensorManager_t *manager, uint8_t slave_addr,
                                          uint8_t func_code, uint16_t reg_addr,
                                          uint16_t reg_count, uint8_t *rx_buf,
                                          int *rx_len, int timeout_ms) {
    uint8_t tx_buf[8];
    tx_buf[0] = slave_addr;
    tx_buf[1] = func_code;
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (reg_count >> 8) & 0xFF;
    tx_buf[5] = reg_count & 0xFF;
    
    uint16_t crc = crc16_modbus(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    
    pthread_mutex_lock(&manager->bus_mutex);
    
    ErrorCode_t ret = rs485_bus_send(tx_buf, 8, timeout_ms);
    if (ret != ERR_OK) {
        pthread_mutex_unlock(&manager->bus_mutex);
        return ret;
    }
    
    ret = rs485_bus_receive(rx_buf, 16, rx_len, timeout_ms);
    pthread_mutex_unlock(&manager->bus_mutex);
    
    if (ret != ERR_OK) {
        return ret;
    }
    
    /* 验证CRC */
    if (*rx_len < 5) {
        return ERR_COMM_FAIL;
    }
    
    uint16_t rx_crc = (rx_buf[*rx_len - 1] << 8) | rx_buf[*rx_len - 2];
    uint16_t calc_crc = crc16_modbus(rx_buf, *rx_len - 2);
    
    if (rx_crc != calc_crc) {
        return ERR_COMM_FAIL;
    }
    
    return ERR_OK;
}

/* 读取并解析编码器数据 - 工业级抗干扰版本 */
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
    
    if (rx_buf[2] != 4) {
        return ERR_COMM_FAIL;
    }
    
    /* 按无符号32位整数解析（手册确认） */
    uint32_t multi_turn_first = ((uint32_t)rx_buf[3] << 24) |
                                ((uint32_t)rx_buf[4] << 16) |
                                ((uint32_t)rx_buf[5] << 8) |
                                ((uint32_t)rx_buf[6]);
    
    /* 检查编码器值是否在有效范围 0~2147483647 */
    if (multi_turn_first > 2147483647) {
        printf("[SENSOR] Invalid encoder value: %u (out of range)\n", multi_turn_first);
        return ERR_COMM_FAIL;
    }
    
    /* 首次读取，建立基准 */
    if (s_encoder_first_read) {
        s_encoder_first_read = 0;
        s_last_valid_encoder = multi_turn_first;
        /* 如果当前zero_offset与读取值相差太远（超过1/4范围），
         * 说明文件保存的零点与当前实际位置跨越了回绕边界，
         * 需要重新校准zero_offset到当前值附近 */
        uint32_t diff = (multi_turn_first > s_encoder_zero_offset) ? 
                        (multi_turn_first - s_encoder_zero_offset) : 
                        (s_encoder_zero_offset - multi_turn_first);
        if (diff > ENCODER_MAX_VALUE / 4) {
            printf("[SENSOR] Recalibrating zero_offset: old=%u, new=%u (diff=%u)\n", 
                   s_encoder_zero_offset, multi_turn_first, diff);
            s_encoder_zero_offset = multi_turn_first;
            s_rope_length_base = 0.0f;
        }
        goto parse_data;
    }
    
    /* 计算与上次有效值的差值 */
    int32_t delta_from_last = calculate_encoder_delta(multi_turn_first, s_last_valid_encoder);
    
    /* 变化正常，接受数据 */
    if (delta_from_last <= ENCODER_MAX_DELTA_PER_CYCLE && 
        delta_from_last >= -ENCODER_MAX_DELTA_PER_CYCLE) {
        s_last_valid_encoder = multi_turn_first;
        s_encoder_consecutive_errors = 0;
        goto parse_data;
    }
    
    /* 检测到突变，重读验证 */
    s_encoder_consecutive_errors++;
    printf("[SENSOR] Suspicious jump: last=%u, current=%u, delta=%d\n",
           s_last_valid_encoder, multi_turn_first, delta_from_last);
    
    usleep(5000);
    
    ret = modbus_read_registers(manager,
                                 manager->configs[SENSOR_TYPE_ENCODER].slave_addr,
                                 manager->configs[SENSOR_TYPE_ENCODER].func_code,
                                 manager->configs[SENSOR_TYPE_ENCODER].reg_addr,
                                 manager->configs[SENSOR_TYPE_ENCODER].reg_count,
                                 rx_buf, &rx_len, 50);
    
    if (ret != ERR_OK) {
        printf("[SENSOR] Retry failed, using last valid\n");
        multi_turn_first = s_last_valid_encoder;
        goto parse_data;
    }
    
    if (rx_buf[2] != 4) {
        multi_turn_first = s_last_valid_encoder;
        goto parse_data;
    }
    
    uint32_t multi_turn_second = ((uint32_t)rx_buf[3] << 24) |
                                 ((uint32_t)rx_buf[4] << 16) |
                                 ((uint32_t)rx_buf[5] << 8) |
                                 ((uint32_t)rx_buf[6]);
    
    if (multi_turn_second > 2147483647) {
        printf("[SENSOR] Retry invalid, using last valid\n");
        multi_turn_first = s_last_valid_encoder;
        goto parse_data;
    }
    
    /* 关键修复：分析两次读取结果 */
    int32_t delta_first_from_last = calculate_encoder_delta(multi_turn_first, s_last_valid_encoder);
    int32_t delta_second_from_last = calculate_encoder_delta(multi_turn_second, s_last_valid_encoder);
    
    /* 判断哪次读取正确 */
    int first_is_normal = (delta_first_from_last <= ENCODER_MAX_DELTA_PER_CYCLE && 
                           delta_first_from_last >= -ENCODER_MAX_DELTA_PER_CYCLE);
    int second_is_normal = (delta_second_from_last <= ENCODER_MAX_DELTA_PER_CYCLE && 
                            delta_second_from_last >= -ENCODER_MAX_DELTA_PER_CYCLE);
    
    if (first_is_normal && !second_is_normal) {
        /* 首次正常，重读异常 - 使用首次 */
        printf("[SENSOR] First OK, second bad, using first: %u\n", multi_turn_first);
        s_last_valid_encoder = multi_turn_first;
        s_encoder_consecutive_errors = 0;
    } else if (!first_is_normal && second_is_normal) {
        /* 首次异常，重读正常 - 使用重读 */
        printf("[SENSOR] First bad, second OK, using second: %u\n", multi_turn_second);
        multi_turn_first = multi_turn_second;
        s_last_valid_encoder = multi_turn_second;
        s_encoder_consecutive_errors = 0;
    } else if (first_is_normal && second_is_normal) {
        /* 两次都正常（非常接近）- 使用第二次 */
        printf("[SENSOR] Both OK, using second: %u\n", multi_turn_second);
        multi_turn_first = multi_turn_second;
        s_last_valid_encoder = multi_turn_second;
        s_encoder_consecutive_errors = 0;
    } else {
        /* 两次都异常（都远离上次有效值） */
        printf("[SENSOR] Both reads abnormal, using last valid: %u\n", s_last_valid_encoder);
        
        if (s_encoder_consecutive_errors >= ENCODER_ERROR_THRESHOLD) {
            printf("[SENSOR] Too many errors, recalibrating...\n");
            s_encoder_zero_offset = multi_turn_second;
            s_rope_length_base = 0.0f;
            s_last_valid_encoder = multi_turn_second;
            s_encoder_consecutive_errors = 0;
            multi_turn_first = multi_turn_second;
        } else {
            multi_turn_first = s_last_valid_encoder;
        }
    }

parse_data:
    /* 填充数据 */
    data->data.encoder.multi_turn_value = multi_turn_first;
    data->data.encoder.angle_deg = (float)multi_turn_first * 360.0f / (float)s_encoder_resolution;
    
    /* 计算相对于零点的脉冲差值（正确处理回绕） */
    int32_t pulse_diff = calculate_encoder_delta(multi_turn_first, s_encoder_zero_offset);
    
    /* 只检查极端异常值（超过1000万脉冲 ≈ 2441圈） */
    if (pulse_diff > 10000000 || pulse_diff < -10000000) {
        printf("[SENSOR] WARNING: Extreme pulse diff: %d, using 0\n", pulse_diff);
        pulse_diff = 0;
    }
    
    float turns = (float)pulse_diff / (float)s_encoder_resolution;
    data->data.encoder.rope_length_mm = s_rope_length_base + turns * s_rope_length_per_turn;
    
    data->data_valid = 1;
    data->last_read_us = get_time_us();
    
    return ERR_OK;
}

/* 读取压力传感器 */
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
    
    if (rx_buf[2] != 2) {
        return ERR_COMM_FAIL;
    }
    
    int16_t raw_value = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
    
    /* 根据传感器配置的小数点位数计算压力值 */
    float divisor = 1.0f;
    uint8_t decimal_places = manager->configs[SENSOR_TYPE_PRESSURE].decimal_places;
    for (uint8_t i = 0; i < decimal_places; i++) {
        divisor *= 10.0f;
    }
    
    float pressure_kg = ((float)(raw_value - s_pressure_zero_offset)) / divisor;
    
    data->data.pressure.raw_value = raw_value;
    data->data.pressure.pressure_kg = pressure_kg;
    data->data_valid = 1;
    data->last_read_us = get_time_us();
    
    return ERR_OK;
}

/* 传感器采集线程 */
static void* sensor_thread(void* arg) {
    SensorManager_t* manager = (SensorManager_t*)arg;
    
    while (manager->running) {
        /* 读取编码器 */
        ErrorCode_t ret = read_encoder(manager, &manager->datas[SENSOR_TYPE_ENCODER]);
        if (ret != ERR_OK) {
            manager->datas[SENSOR_TYPE_ENCODER].error_count++;
        } else {
            manager->datas[SENSOR_TYPE_ENCODER].read_count++;
        }
        
        /* 读取压力 */
        ret = read_pressure(manager, &manager->datas[SENSOR_TYPE_PRESSURE]);
        if (ret != ERR_OK) {
            manager->datas[SENSOR_TYPE_PRESSURE].error_count++;
        } else {
            manager->datas[SENSOR_TYPE_PRESSURE].read_count++;
        }
        
        manager->cycle_count++;
        usleep(10000); /* 10ms = 100Hz */
    }
    
    return NULL;
}

/* 加载编码器数据 */
static int load_encoder_data(void) {
    FILE *fp = fopen(ENCODER_DATA_FILE, "r");
    if (fp == NULL) {
        printf("[SENSOR] No encoder data file\n");
        return -1;
    }
    
    char line[128];
    float saved_length = 0.0f;
    uint32_t saved_encoder = 0;
    int has_data = 0;
    
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#') continue;
        
        if (strncmp(line, "ABSOLUTE_LENGTH_MM=", 19) == 0) {
            sscanf(line + 19, "%f", &saved_length);
            has_data = 1;
        }
        else if (strncmp(line, "LAST_ENCODER_VALUE=", 19) == 0) {
            sscanf(line + 19, "%u", &saved_encoder);
            has_data = 1;
        }
        else if (strncmp(line, "DRUM_DIAMETER=", 14) == 0) {
            sscanf(line + 14, "%f", &s_rope_drum_diameter);
        }
        else if (strncmp(line, "ENCODER_RESOLUTION=", 19) == 0) {
            sscanf(line + 19, "%u", &s_encoder_resolution);
        }
    }
    
    fclose(fp);
    
    s_rope_length_per_turn = M_PI * s_rope_drum_diameter;
    
    if (has_data) {
        printf("[SENSOR] Loaded: length=%.2fmm, encoder=%u\n", saved_length, saved_encoder);
        s_rope_length_base = saved_length;
        s_encoder_zero_offset = saved_encoder;
    }
    
    return 0;
}

/* 保存编码器数据 */
static int save_encoder_data(uint32_t encoder_value, float absolute_length) {
    FILE *fp = fopen(ENCODER_DATA_FILE, "w");
    if (fp == NULL) {
        return -1;
    }
    
    fprintf(fp, "# Encoder Rope Length Data\n");
    fprintf(fp, "ABSOLUTE_LENGTH_MM=%.4f\n", absolute_length);
    fprintf(fp, "LAST_ENCODER_VALUE=%u\n", encoder_value);
    fprintf(fp, "DRUM_DIAMETER=%.4f\n", s_rope_drum_diameter);
    fprintf(fp, "ENCODER_RESOLUTION=%u\n", s_encoder_resolution);
    
    fclose(fp);
    printf("[SENSOR] Saved: length=%.2fmm, encoder=%u\n", absolute_length, encoder_value);
    return 0;
}

/* 初始化传感器管理器 */
ErrorCode_t sensor_mgr_init(SensorManager_t *manager, const char *device, int baudrate) {
    if (manager == NULL || device == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(manager, 0, sizeof(SensorManager_t));
    
    /* 初始化互斥锁 */
    pthread_mutex_init(&manager->bus_mutex, NULL);
    
    /* 配置编码器 */
    manager->configs[SENSOR_TYPE_ENCODER].slave_addr = ENCODER_SLAVE_ADDR;
    manager->configs[SENSOR_TYPE_ENCODER].func_code = ENCODER_MODBUS_FUNC_CODE;
    manager->configs[SENSOR_TYPE_ENCODER].reg_addr = ENCODER_MODBUS_REG_ADDR;
    manager->configs[SENSOR_TYPE_ENCODER].reg_count = 2;
    manager->configs[SENSOR_TYPE_ENCODER].name = "Encoder";
    
    /* 配置压力传感器 */
    manager->configs[SENSOR_TYPE_PRESSURE].slave_addr = PRESSURE_SLAVE_ADDR;
    manager->configs[SENSOR_TYPE_PRESSURE].func_code = 0x03;  /* 读取保持寄存器 */
    manager->configs[SENSOR_TYPE_PRESSURE].reg_addr = 0x0000;  /* 压力值寄存器 */
    manager->configs[SENSOR_TYPE_PRESSURE].reg_count = 1;
    manager->configs[SENSOR_TYPE_PRESSURE].decimal_places = 2;  /* 2位小数 = 0.01kg分辨率 */
    manager->configs[SENSOR_TYPE_PRESSURE].name = "Pressure";
    
    /* 加载保存的数据 */
    load_encoder_data();
    
    /* 初始化RS485 */
    ErrorCode_t ret = rs485_bus_init(device, baudrate);
    if (ret != ERR_OK) {
        printf("[SENSOR] Failed to init RS485: %d\n", ret);
        return ret;
    }
    
    manager->rs485_fd = rs485_bus_get_fd();
    manager->initialized = 1;
    printf("[SENSOR] Initialized\n");
    
    return ERR_OK;
}

/* 启动传感器管理器 */
ErrorCode_t sensor_mgr_start(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    manager->running = 1;
    
    if (pthread_create(&manager->manager_thread, NULL, sensor_thread, manager) != 0) {
        return ERR_GENERAL;
    }
    
    printf("[SENSOR] Started\n");
    return ERR_OK;
}

/* 停止传感器管理器 */
void sensor_mgr_stop(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return;
    }
    
    manager->running = 0;
    pthread_join(manager->manager_thread, NULL);
    
    printf("[SENSOR] Stopped\n");
}

/* 反初始化 */
void sensor_mgr_deinit(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return;
    }
    
    /* 停止线程 */
    manager->running = 0;
    pthread_join(manager->manager_thread, NULL);
    
    /* 保存当前位置 */
    SensorData_t *encoder_data = &manager->datas[SENSOR_TYPE_ENCODER];
    if (encoder_data->data_valid) {
        uint32_t current = encoder_data->data.encoder.multi_turn_value;
        int32_t delta = calculate_encoder_delta(current, s_encoder_zero_offset);
        float turns = (float)delta / (float)s_encoder_resolution;
        float absolute = s_rope_length_base + turns * s_rope_length_per_turn;
        save_encoder_data(current, absolute);
    }
    
    rs485_bus_deinit();
    pthread_mutex_destroy(&manager->bus_mutex);
    manager->initialized = 0;
}

/* 获取传感器数据 */
ErrorCode_t sensor_mgr_get_data(SensorManager_t *manager, SensorType_t type, SensorData_t *data) {
    if (manager == NULL || data == NULL || type >= SENSOR_TYPE_COUNT) {
        return ERR_INVALID_PARAM;
    }
    
    if (!manager->initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    pthread_mutex_lock(&manager->bus_mutex);
    *data = manager->datas[type];
    pthread_mutex_unlock(&manager->bus_mutex);
    
    return ERR_OK;
}

/* 设置编码器绳子长度参数 */
ErrorCode_t sensor_mgr_set_encoder_rope_params(SensorManager_t *manager, 
                                                float drum_diameter, 
                                                uint32_t resolution) {
    if (manager == NULL || drum_diameter <= 0 || resolution == 0) {
        return ERR_INVALID_PARAM;
    }
    
    s_rope_drum_diameter = drum_diameter;
    s_encoder_resolution = resolution;
    s_rope_length_per_turn = M_PI * drum_diameter;
    
    return ERR_OK;
}

/* 执行编码器零点校准 */
ErrorCode_t sensor_mgr_encoder_zero_calibration(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    SensorData_t data;
    ErrorCode_t ret = sensor_mgr_get_data(manager, SENSOR_TYPE_ENCODER, &data);
    if (ret != ERR_OK) {
        return ret;
    }
    
    s_encoder_zero_offset = data.data.encoder.multi_turn_value;
    s_rope_length_base = 0.0f;
    
    printf("[SENSOR] Encoder zero calibrated: offset=%u\n", s_encoder_zero_offset);
    return ERR_OK;
}

/* 执行压力传感器去皮/清零 */
ErrorCode_t sensor_mgr_pressure_tare(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    SensorData_t data;
    ErrorCode_t ret = sensor_mgr_get_data(manager, SENSOR_TYPE_PRESSURE, &data);
    if (ret != ERR_OK) {
        return ret;
    }
    
    s_pressure_zero_offset = data.data.pressure.raw_value;
    
    printf("[SENSOR] Pressure tared: offset=%d\n", s_pressure_zero_offset);
    return ERR_OK;
}

/* 设置编码器基准长度 */
ErrorCode_t sensor_mgr_set_encoder_base_length(SensorManager_t *manager, float base_length_mm) {
    if (manager == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    s_rope_length_base = base_length_mm;
    return ERR_OK;
}

/* 获取传感器读取成功率 */
float sensor_mgr_get_success_rate(SensorManager_t *manager, SensorType_t type) {
    if (manager == NULL || type >= SENSOR_TYPE_COUNT) {
        return 0.0f;
    }
    
    uint32_t reads = manager->datas[type].read_count;
    uint32_t errors = manager->datas[type].error_count;
    uint32_t total = reads + errors;
    
    if (total == 0) {
        return 0.0f;
    }
    
    return (float)reads * 100.0f / (float)total;
}

/* 打印所有传感器状态 */
void sensor_mgr_print_status(SensorManager_t *manager) {
    if (manager == NULL) {
        return;
    }
    
    printf("\n=== Sensor Manager Status ===\n");
    printf("Cycle count: %lu\n", manager->cycle_count);
    printf("Total errors: %lu\n", manager->total_errors);
    
    const char* type_names[] = {"Encoder", "Pressure"};
    
    for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
        SensorData_t *data = &manager->datas[i];
        float success_rate = sensor_mgr_get_success_rate(manager, i);
        
        printf("\n%s (addr=%d):\n", type_names[i], manager->configs[i].slave_addr);
        printf("  Reads: %lu, Errors: %lu, Success: %.1f%%\n",
               data->read_count, data->error_count, success_rate);
        
        if (i == SENSOR_TYPE_ENCODER && data->data_valid) {
            printf("  Multi-turn: %u\n", data->data.encoder.multi_turn_value);
            printf("  Angle: %.2f deg\n", data->data.encoder.angle_deg);
            printf("  Rope: %.2f mm\n", data->data.encoder.rope_length_mm);
        } else if (i == SENSOR_TYPE_PRESSURE && data->data_valid) {
            printf("  Pressure: %.3f kg (raw=%d, offset=%d)\n",
                   data->data.pressure.pressure_kg,
                   data->data.pressure.raw_value,
                   s_pressure_zero_offset);
        }
    }
}
