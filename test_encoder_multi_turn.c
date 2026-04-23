/******************************************************************************
 * @file    test_encoder_multi_turn.c
 * @brief   编码器虚拟多圈值和绳子长度测试程序
 * @author  System Architect
 * @date    2026-04-23
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include "drivers/encoder_driver.h"
#include "drivers/rs485_bus.h"
#include "utils/logger.h"
#include "config/system_config.h"

/* 测试配置 */
#define TEST_DURATION_SECONDS   30
#define DATA_SAVE_FILE          "/tmp/encoder_rope_data.txt"

static volatile int running = 1;

void signal_handler(int sig) {
    running = 0;
    printf("\nTest interrupted by signal %d\n", sig);
}

int main(int argc, char *argv[]) {
    (void)argc;  /* 未使用参数 */
    (void)argv;
    
    printf("========================================\n");
    printf("Encoder Multi-Turn & Rope Length Test\n");
    printf("========================================\n\n");
    
    /* 设置信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* 初始化日志 */
    logger_init(NULL, NULL, LOG_LEVEL_INFO, 1);
    
    /* 初始化RS485总线 */
    printf("Initializing RS485 bus...\n");
    ErrorCode_t ret = rs485_bus_init(ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE);
    if (ret != ERR_OK) {
        printf("ERROR: Failed to initialize RS485 bus\n");
        return 1;
    }
    printf("RS485 bus initialized: %s @ %d baud\n\n", 
           ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE);
    
    /* 初始化编码器驱动 */
    printf("Initializing encoder driver...\n");
    EncoderDriver_t encoder;
    ret = encoder_init(&encoder, ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE, ENCODER_SLAVE_ADDR);
    if (ret != ERR_OK) {
        printf("ERROR: Failed to initialize encoder driver\n");
        rs485_bus_deinit();
        return 1;
    }
    printf("Encoder driver initialized: addr=%d, resolution=%d\n\n",
           ENCODER_SLAVE_ADDR, ENCODER_RESOLUTION);
    
    /* 步骤1: 尝试加载之前保存的绳子长度数据 */
    printf("Step 1: Loading saved rope length data...\n");
    ret = encoder_load_rope_length(&encoder, DATA_SAVE_FILE);
    if (ret == ERR_OK) {
        printf("Loaded previous rope length data from %s\n", DATA_SAVE_FILE);
    } else {
        printf("No previous data found, using default values\n");
    }
    
    /* 步骤2: 设置卷筒参数 (直径50mm) */
    printf("\nStep 2: Setting rope drum parameters...\n");
    float drum_diameter = 50.0f;  /* 卷筒直径50mm */
    ret = encoder_set_rope_params(&encoder, drum_diameter, encoder.rope_length_base);
    if (ret != ERR_OK) {
        printf("ERROR: Failed to set rope parameters\n");
        encoder_deinit(&encoder);
        rs485_bus_deinit();
        return 1;
    }
    printf("Drum diameter: %.2f mm\n", drum_diameter);
    printf("Rope length per turn: %.2f mm\n", encoder.rope_length_per_turn);
    printf("Base length (from file): %.2f mm\n\n", encoder.rope_length_base);
    
    /* 步骤3: 执行零点校准 */
    printf("Step 3: Performing zero calibration...\n");
    ret = encoder_zero_calibration(&encoder);
    if (ret != ERR_OK) {
        printf("ERROR: Zero calibration failed\n");
        encoder_deinit(&encoder);
        rs485_bus_deinit();
        return 1;
    }
    printf("Zero calibration completed!\n");
    printf("Current multi-turn value saved as zero offset\n\n");
    
    /* 步骤4: 读取并显示多圈值 */
    printf("Step 4: Reading multi-turn value...\n");
    uint32_t multi_turn_value;
    ret = encoder_read_multi_turn(&encoder, &multi_turn_value);
    if (ret != ERR_OK) {
        printf("ERROR: Failed to read multi-turn value\n");
        encoder_deinit(&encoder);
        rs485_bus_deinit();
        return 1;
    }
    printf("Current multi-turn value: %u\n", multi_turn_value);
    printf("Zero offset: %u\n", encoder.multi_turn_zero_offset);
    printf("Relative turns: %d\n", 
           (int32_t)(multi_turn_value - encoder.multi_turn_zero_offset));
    
    /* 步骤5: 计算多圈角度 */
    printf("\nStep 5: Calculating multi-turn angle...\n");
    float angle_deg;
    ret = encoder_get_multi_turn_angle(&encoder, &angle_deg);
    if (ret != ERR_OK) {
        printf("ERROR: Failed to get multi-turn angle\n");
    } else {
        printf("Multi-turn angle: %.2f degrees\n", angle_deg);
        printf("Equivalent turns: %.2f\n", angle_deg / 360.0f);
    }
    
    /* 步骤6: 计算当前绳子长度 */
    printf("\nStep 6: Calculating rope length...\n");
    float rope_length;
    ret = encoder_calc_rope_length(&encoder, &rope_length);
    if (ret != ERR_OK) {
        printf("ERROR: Failed to calculate rope length\n");
    } else {
        printf("Current rope length: %.2f mm\n", rope_length);
        printf("Current rope length: %.4f m\n", rope_length / 1000.0f);
    }
    
    /* 步骤7: 持续监测 */
    printf("\n========================================\n");
    printf("Continuous monitoring for %d seconds...\n", TEST_DURATION_SECONDS);
    printf("Press Ctrl+C to stop\n");
    printf("========================================\n\n");
    
    int counter = 0;
    while (running && counter < TEST_DURATION_SECONDS) {
        /* 读取多圈值 */
        ret = encoder_read_multi_turn(&encoder, &multi_turn_value);
        if (ret != ERR_OK) {
            printf("[ERROR] Failed to read multi-turn value\n");
            usleep(1000000);
            counter++;
            continue;
        }
        
        /* 计算绳子长度 */
        ret = encoder_calc_rope_length(&encoder, &rope_length);
        if (ret != ERR_OK) {
            printf("[ERROR] Failed to calculate rope length\n");
            usleep(1000000);
            counter++;
            continue;
        }
        
        /* 计算角度 */
        float current_angle = (float)multi_turn_value * 360.0f / (float)encoder.resolution;
        
        /* 每5秒显示一次 */
        if (counter % 5 == 0) {
            printf("[%02d s] Multi-turn: %10u | Angle: %8.2f deg | Rope: %8.2f mm (%6.4f m)\n",
                   counter,
                   multi_turn_value,
                   current_angle,
                   rope_length,
                   rope_length / 1000.0f);
        }
        
        usleep(1000000);  /* 1秒间隔 */
        counter++;
    }
    
    /* 步骤8: 保存当前绳子长度 */
    printf("\n========================================\n");
    printf("Step 8: Saving rope length data...\n");
    printf("========================================\n");
    
    ret = encoder_save_rope_length(&encoder, DATA_SAVE_FILE);
    if (ret != ERR_OK) {
        printf("ERROR: Failed to save rope length data\n");
    } else {
        printf("Rope length data saved to: %s\n", DATA_SAVE_FILE);
        
        /* 显示保存的数据 */
        printf("\nSaved data:\n");
        printf("  Base length: %.2f mm\n", encoder.rope_length_base);
        printf("  Zero offset: %u\n", encoder.multi_turn_zero_offset);
        printf("  Drum diameter: %.2f mm\n", encoder.rope_drum_diameter);
        printf("  Resolution: %u\n", encoder.resolution);
    }
    
    /* 清理 */
    printf("\nCleaning up...\n");
    encoder_deinit(&encoder);
    rs485_bus_deinit();
    
    printf("\nTest completed!\n");
    printf("\n========================================\n");
    printf("Usage Instructions:\n");
    printf("========================================\n");
    printf("1. The system automatically saves rope length to:\n");
    printf("   %s\n", DATA_SAVE_FILE);
    printf("2. On next startup, the system loads this data\n");
    printf("3. Zero calibration is performed at each startup\n");
    printf("4. Current length = Base + (Current - Offset) x PerTurn\n");
    printf("========================================\n");
    
    return 0;
}
