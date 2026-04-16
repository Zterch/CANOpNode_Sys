/******************************************************************************
 * @file    sine_wave.c
 * @brief   正弦波生成算法实现
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#include "sine_wave.h"
#include <stddef.h>

void sine_wave_init(SineWaveGenerator_t *gen, double amplitude, 
                    double frequency, double phase, double offset, double dt) {
    if (gen == NULL) {
        return;
    }
    
    gen->amplitude = amplitude;
    gen->frequency = frequency;
    gen->phase = phase;
    gen->offset = offset;
    gen->dt = dt;
    gen->time = 0.0;
}

double sine_wave_generate(SineWaveGenerator_t *gen) {
    if (gen == NULL) {
        return 0.0;
    }
    
    double value = gen->amplitude * sin(2.0 * M_PI * gen->frequency * gen->time + gen->phase) 
                   + gen->offset;
    
    gen->time += gen->dt;
    
    return value;
}

void sine_wave_reset(SineWaveGenerator_t *gen) {
    if (gen == NULL) {
        return;
    }
    
    gen->time = 0.0;
}

void sine_wave_set_amplitude(SineWaveGenerator_t *gen, double amplitude) {
    if (gen == NULL) {
        return;
    }
    
    gen->amplitude = amplitude;
}

void sine_wave_set_frequency(SineWaveGenerator_t *gen, double frequency) {
    if (gen == NULL) {
        return;
    }
    
    gen->frequency = frequency;
}
