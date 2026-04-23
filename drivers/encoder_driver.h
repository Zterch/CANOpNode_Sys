/******************************************************************************
 * @file    encoder_driver.h
 * @brief   编码器驱动 - RS485接口绝对值编码器 (Modbus RTU)
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#ifndef __ENCODER_DRIVER_H__
#define __ENCODER_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 编码器数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    char device[32];            /* 串口设备 */
    int baudrate;               /* 波特率 */
    uint8_t slave_addr;         /* RS485设备地址 */
    int fd;                     /* 串口文件描述符 */
    uint32_t resolution;        /* 分辨率 */
    
    /* 单圈数据 */
    int32_t position;           /* 位置值 */
    int32_t velocity;           /* 速度值 */
    int32_t last_position;      /* 上次位置值 */
    clock_t timestamp;          /* 时间戳 */
    clock_t last_time;          /* 上次时间 */
    
    /* 多圈数据 (虚拟多圈值) */
    uint32_t multi_turn_value;      /* 当前虚拟多圈值 */
    uint32_t multi_turn_zero_offset;/* 零点偏移 (系统启动时的多圈值) */
    float multi_turn_angle;         /* 多圈角度值 */
    
    /* 绳子长度计算 */
    float rope_length_base;         /* 绳子基准长度 (保存值) */
    float rope_length_per_turn;     /* 每圈绳子长度 (mm/圈) */
    float rope_length_current;      /* 当前绳子长度 */
    float rope_drum_diameter;       /* 卷筒直径 (mm) */
    
    /* 统计 */
    uint32_t read_count;        /* 读取次数 */
    uint32_t error_count;       /* 错误次数 */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    int initialized;
} EncoderDriver_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化编码器驱动
 * @param encoder 编码器驱动结构指针
 * @param device 串口设备
 * @param baudrate 波特率
 * @param slave_addr 设备地址
 * @return ErrorCode_t
 */
ErrorCode_t encoder_init(EncoderDriver_t *encoder, const char *device, 
                         int baudrate, uint8_t slave_addr);

/**
 * @brief 反初始化编码器驱动
 * @param encoder 编码器驱动结构指针
 */
void encoder_deinit(EncoderDriver_t *encoder);

/**
 * @brief 读取编码器位置 (Modbus RTU协议)
 * @param encoder 编码器驱动结构指针
 * @param position 位置输出指针
 * @return ErrorCode_t
 */
ErrorCode_t encoder_read_position(EncoderDriver_t *encoder, int32_t *position);

/**
 * @brief 读取编码器速度
 * @param encoder 编码器驱动结构指针
 * @param velocity 速度输出指针
 * @return ErrorCode_t
 */
ErrorCode_t encoder_read_velocity(EncoderDriver_t *encoder, int32_t *velocity);

/**
 * @brief 获取最后一次成功读取的位置值
 * @param encoder 编码器驱动结构指针
 * @param position 位置输出指针
 * @param timestamp 时间戳输出指针（可为NULL）
 * @return ErrorCode_t
 */
ErrorCode_t encoder_get_last_position(EncoderDriver_t *encoder, int32_t *position, clock_t *timestamp);

/**
 * @brief 将编码器数值转换为角度
 * @param position 编码器位置值 (0 ~ resolution-1)
 * @param resolution 编码器分辨率 (如4096)
 * @return 角度值 (0.0 ~ 360.0)
 */
static inline float encoder_position_to_angle(uint16_t position, uint16_t resolution) {
    if (resolution == 0) return 0.0f;
    return (float)position * 360.0f / (float)resolution;
}

/**
 * @brief 读取编码器虚拟多圈值 (32位无符号整数)
 * @param encoder 编码器驱动结构指针
 * @param multi_turn_value 多圈值输出指针
 * @return ErrorCode_t
 * 
 * 说明：读取寄存器0x0000-0x0001，返回32位虚拟多圈值
 * 计算公式：角度 = 多圈值 * 360 / 单圈分辨率
 */
ErrorCode_t encoder_read_multi_turn(EncoderDriver_t *encoder, uint32_t *multi_turn_value);

/**
 * @brief 设置绳子长度参数
 * @param encoder 编码器驱动结构指针
 * @param drum_diameter 卷筒直径 (mm)
 * @param length_base 绳子基准长度 (mm, 从文件加载的值)
 * @return ErrorCode_t
 */
ErrorCode_t encoder_set_rope_params(EncoderDriver_t *encoder, float drum_diameter, float length_base);

/**
 * @brief 执行零点校准 (系统启动时调用)
 * @param encoder 编码器驱动结构指针
 * @return ErrorCode_t
 * 
 * 说明：记录当前编码器值作为零点偏移，解决掉电归零问题
 */
ErrorCode_t encoder_zero_calibration(EncoderDriver_t *encoder);

/**
 * @brief 计算当前绳子长度
 * @param encoder 编码器驱动结构指针
 * @param length_mm 绳子长度输出指针 (mm)
 * @return ErrorCode_t
 * 
 * 计算公式：当前长度 = 基准长度 + (当前多圈值 - 零点偏移) * 每圈绳长
 */
ErrorCode_t encoder_calc_rope_length(EncoderDriver_t *encoder, float *length_mm);

/**
 * @brief 保存当前绳子长度到文件
 * @param encoder 编码器驱动结构指针
 * @param filename 保存文件路径
 * @return ErrorCode_t
 */
ErrorCode_t encoder_save_rope_length(EncoderDriver_t *encoder, const char *filename);

/**
 * @brief 从文件加载绳子基准长度
 * @param encoder 编码器驱动结构指针
 * @param filename 加载文件路径
 * @return ErrorCode_t
 */
ErrorCode_t encoder_load_rope_length(EncoderDriver_t *encoder, const char *filename);

/**
 * @brief 获取多圈角度值
 * @param encoder 编码器驱动结构指针
 * @param angle_deg 角度输出指针 (度)
 * @return ErrorCode_t
 */
ErrorCode_t encoder_get_multi_turn_angle(EncoderDriver_t *encoder, float *angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_DRIVER_H__ */
