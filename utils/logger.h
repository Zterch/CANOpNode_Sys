/******************************************************************************
 * @file    logger.h
 * @brief   日志系统 - 线程安全的日志记录
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <stdio.h>
#include <stdarg.h>
#include <pthread.h>
#include <time.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 日志级别定义
 ******************************************************************************/
typedef enum {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_WARN = 1,
    LOG_LEVEL_INFO = 2,
    LOG_LEVEL_DEBUG = 3,
    LOG_LEVEL_TRACE = 4
} LogLevel_t;

/******************************************************************************
 * 日志模块定义
 ******************************************************************************/
typedef enum {
    LOG_MODULE_SYS = 0,     /* 系统模块 */
    LOG_MODULE_MOTOR,       /* 电机模块 */
    LOG_MODULE_POWER,       /* 电源模块 */
    LOG_MODULE_ENCODER,     /* 编码器模块 */
    LOG_MODULE_PRESSURE,    /* 压力计模块 */
    LOG_MODULE_ALGO,        /* 算法模块 */
    LOG_MODULE_DRV,         /* 驱动模块 */
    LOG_MODULE_NUM          /* 模块数量 */
} LogModule_t;

/******************************************************************************
 * 日志句柄结构
 ******************************************************************************/
typedef struct {
    FILE *fp;
    LogLevel_t level;
    pthread_mutex_t mutex;
    char filename[256];
    int enable_console;
    int enable_file;
} Logger_t;

/******************************************************************************
 * 全局日志实例
 ******************************************************************************/
extern Logger_t g_logger;

/******************************************************************************
 * 日志函数宏
 ******************************************************************************/
#define LOG_ERROR(module, fmt, ...) \
    logger_log(&g_logger, LOG_LEVEL_ERROR, module, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define LOG_WARN(module, fmt, ...) \
    logger_log(&g_logger, LOG_LEVEL_WARN, module, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define LOG_INFO(module, fmt, ...) \
    logger_log(&g_logger, LOG_LEVEL_INFO, module, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define LOG_DEBUG(module, fmt, ...) \
    logger_log(&g_logger, LOG_LEVEL_DEBUG, module, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

#define LOG_TRACE(module, fmt, ...) \
    logger_log(&g_logger, LOG_LEVEL_TRACE, module, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化日志系统
 * @param logger 日志句柄
 * @param filename 日志文件名（NULL表示不输出到文件）
 * @param level 日志级别
 * @param enable_console 是否输出到控制台
 * @return ErrorCode_t
 */
ErrorCode_t logger_init(Logger_t *logger, const char *filename, 
                        LogLevel_t level, int enable_console);

/**
 * @brief 关闭日志系统
 * @param logger 日志句柄
 */
void logger_deinit(Logger_t *logger);

/**
 * @brief 记录日志
 * @param logger 日志句柄
 * @param level 日志级别
 * @param module 模块标识
 * @param file 源文件名
 * @param line 行号
 * @param fmt 格式字符串
 * @param ... 可变参数
 */
void logger_log(Logger_t *logger, LogLevel_t level, LogModule_t module,
                const char *file, int line, const char *fmt, ...);

/**
 * @brief 设置日志级别
 * @param logger 日志句柄
 * @param level 日志级别
 */
void logger_set_level(Logger_t *logger, LogLevel_t level);

/**
 * @brief 获取模块名称字符串
 * @param module 模块标识
 * @return 模块名称字符串
 */
const char* logger_get_module_name(LogModule_t module);

/**
 * @brief 获取日志级别字符串
 * @param level 日志级别
 * @return 级别名称字符串
 */
const char* logger_get_level_name(LogLevel_t level);

#ifdef __cplusplus
}
#endif

#endif /* __LOGGER_H__ */
