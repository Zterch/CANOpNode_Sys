/******************************************************************************
 * @file    test_encoder_simple.c
 * @brief   编码器简单测试 - 验证数据读取和计算
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "drivers/encoder_driver.h"
#include "drivers/rs485_bus.h"
#include "utils/logger.h"
#include "config/system_config.h"

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    
    printf("========================================\n");
    printf("Simple Encoder Test\n");
    printf("========================================\n\n");
    
    logger_init(NULL, NULL, LOG_LEVEL_INFO, 1);
    
    /* 初始化RS485 */
    if (rs485_bus_init(ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE) != ERR_OK) {
        printf("ERROR: Failed to init RS485\n");
        return 1;
    }
    
    /* 初始化编码器 */
    EncoderDriver_t encoder;
    if (encoder_init(&encoder, ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE, ENCODER_SLAVE_ADDR) != ERR_OK) {
        printf("ERROR: Failed to init encoder\n");
        rs485_bus_deinit();
        return 1;
    }
    
    printf("Encoder initialized (addr=%d, resolution=%d)\n\n", ENCODER_SLAVE_ADDR, ENCODER_RESOLUTION);
    
    /* 设置卷筒参数 */
    encoder_set_rope_params(&encoder, 100.0f, 0.0f);
    printf("Drum diameter: 100mm, Per turn: %.2fmm\n\n", encoder.rope_length_per_turn);
    
    /* 读取初始值 */
    uint32_t multi_turn;
    float angle;
    
    printf("Step 1: Reading initial value...\n");
    if (encoder_read_multi_turn(&encoder, &multi_turn) == ERR_OK) {
        encoder_get_multi_turn_angle(&encoder, &angle);
        printf("  Initial MultiTurn: %u\n", multi_turn);
        printf("  Initial Angle: %.2f degrees\n", angle);
        printf("  Initial Turns: %.2f\n\n", (float)multi_turn / ENCODER_RESOLUTION);
    } else {
        printf("  ERROR: Failed to read initial value\n\n");
    }
    
    /* 零点校准 */
    printf("Step 2: Zero calibration...\n");
    if (encoder_zero_calibration(&encoder) == ERR_OK) {
        printf("  Zero offset: %u\n\n", encoder.multi_turn_zero_offset);
    } else {
        printf("  ERROR: Zero calibration failed\n\n");
    }
    
    /* 持续读取并显示变化 */
    printf("Step 3: Continuous reading (10 seconds)...\n");
    printf("Rotate the encoder slowly and watch the values...\n\n");
    
    uint32_t last_value = encoder.multi_turn_zero_offset;
    float last_length = 0.0f;
    
    for (int i = 0; i < 10; i++) {
        if (encoder_read_multi_turn(&encoder, &multi_turn) == ERR_OK) {
            encoder_get_multi_turn_angle(&encoder, &angle);
            
            /* 计算相对变化 */
            int64_t delta = (int64_t)multi_turn - (int64_t)encoder.multi_turn_zero_offset;
            float turns = (float)delta / ENCODER_RESOLUTION;
            float length = delta * encoder.rope_length_per_turn;
            float delta_length = length - last_length;
            
            printf("[%2d] MultiTurn=%6u | Delta=%6ld | Turns=%7.2f | Length=%8.2fmm | Change=%7.2fmm\n",
                   i, multi_turn, (long)delta, turns, length, delta_length);
            
            last_value = multi_turn;
            last_length = length;
        } else {
            printf("[%2d] ERROR: Read failed\n", i);
        }
        
        sleep(1);
    }
    
    /* 保存测试 */
    printf("\nStep 4: Saving data...\n");
    if (encoder_save_rope_length(&encoder, "/tmp/test_rope_data.txt") == ERR_OK) {
        printf("  Data saved to /tmp/test_rope_data.txt\n");
        
        /* 显示文件内容 */
        FILE *fp = fopen("/tmp/test_rope_data.txt", "r");
        if (fp) {
            char line[256];
            printf("  File contents:\n");
            while (fgets(line, sizeof(line), fp)) {
                if (line[0] != '#') {
                    printf("    %s", line);
                }
            }
            fclose(fp);
        }
    }
    
    /* 清理 */
    encoder_deinit(&encoder);
    rs485_bus_deinit();
    
    printf("\nTest completed!\n");
    return 0;
}
