/******************************************************************************
 * @file    sine_wave.h
 * @brief   正弦波生成算法 - 用于电机速度控制
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#ifndef __SINE_WAVE_H__
#define __SINE_WAVE_H__

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 正弦波生成器结构
 ******************************************************************************/
typedef struct {
    double amplitude;           /* 振幅 */
    double frequency;           /* 频率 (Hz) */
    double phase;               /* 初始相位 (rad) */
    double offset;              /* 直流偏移 */
    double time;                /* 当前时间 (s) */
    double dt;                  /* 时间步长 (s) */
} SineWaveGenerator_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化正弦波生成器
 * @param gen 生成器指针
 * @param amplitude 振幅
 * @param frequency 频率 (Hz)
 * @param phase 初始相位 (rad)
 * @param offset 直流偏移
 * @param dt 时间步长 (s)
 */
void sine_wave_init(SineWaveGenerator_t *gen, double amplitude, 
                    double frequency, double phase, double offset, double dt);

/**
 * @brief 生成下一个正弦波值
 * @param gen 生成器指针
 * @return 当前正弦波值
 */
double sine_wave_generate(SineWaveGenerator_t *gen);

/**
 * @brief 重置生成器
 * @param gen 生成器指针
 */
void sine_wave_reset(SineWaveGenerator_t *gen);

/**
 * @brief 设置振幅
 * @param gen 生成器指针
 * @param amplitude 新振幅
 */
void sine_wave_set_amplitude(SineWaveGenerator_t *gen, double amplitude);

/**
 * @brief 设置频率
 * @param gen 生成器指针
 * @param frequency 新频率 (Hz)
 */
void sine_wave_set_frequency(SineWaveGenerator_t *gen, double frequency);

#ifdef __cplusplus
}
#endif

#endif /* __SINE_WAVE_H__ */
