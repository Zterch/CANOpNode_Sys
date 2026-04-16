/******************************************************************************
 * @file    logger.c
 * @brief   日志系统实现
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#include "logger.h"
#include <string.h>
#include <stdlib.h>

/* 全局日志实例 */
Logger_t g_logger = {0};

/* 模块名称映射表 */
static const char *module_names[LOG_MODULE_NUM] = {
    [LOG_MODULE_SYS]     = "SYS",
    [LOG_MODULE_MOTOR]   = "MOTOR",
    [LOG_MODULE_POWER]   = "POWER",
    [LOG_MODULE_ENCODER] = "ENCODER",
    [LOG_MODULE_PRESSURE]= "PRESSURE",
    [LOG_MODULE_ALGO]    = "ALGO",
    [LOG_MODULE_DRV]     = "DRV"
};

/* 日志级别名称映射表 */
static const char *level_names[] = {
    [LOG_LEVEL_ERROR] = "ERROR",
    [LOG_LEVEL_WARN]  = "WARN",
    [LOG_LEVEL_INFO]  = "INFO",
    [LOG_LEVEL_DEBUG] = "DEBUG",
    [LOG_LEVEL_TRACE] = "TRACE"
};

/* 日志级别颜色映射（ANSI转义码）*/
static const char *level_colors[] = {
    [LOG_LEVEL_ERROR] = "\033[31m",  /* 红色 */
    [LOG_LEVEL_WARN]  = "\033[33m",  /* 黄色 */
    [LOG_LEVEL_INFO]  = "\033[32m",  /* 绿色 */
    [LOG_LEVEL_DEBUG] = "\033[36m",  /* 青色 */
    [LOG_LEVEL_TRACE] = "\033[37m"   /* 白色 */
};

static const char *color_reset = "\033[0m";

/******************************************************************************
 * 函数实现
 ******************************************************************************/

ErrorCode_t logger_init(Logger_t *logger, const char *filename, 
                        LogLevel_t level, int enable_console) {
    if (logger == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    /* 初始化互斥锁 */
    if (pthread_mutex_init(&logger->mutex, NULL) != 0) {
        return ERR_GENERAL;
    }
    
    logger->level = level;
    logger->enable_console = enable_console;
    logger->enable_file = 0;
    logger->fp = NULL;
    
    /* 打开日志文件 */
    if (filename != NULL && strlen(filename) > 0) {
        strncpy(logger->filename, filename, sizeof(logger->filename) - 1);
        logger->fp = fopen(filename, "a");
        if (logger->fp == NULL) {
            pthread_mutex_destroy(&logger->mutex);
            return ERR_GENERAL;
        }
        logger->enable_file = 1;
    }
    
    return ERR_OK;
}

void logger_deinit(Logger_t *logger) {
    if (logger == NULL) {
        return;
    }
    
    pthread_mutex_lock(&logger->mutex);
    
    if (logger->fp != NULL) {
        fclose(logger->fp);
        logger->fp = NULL;
    }
    
    logger->enable_file = 0;
    logger->enable_console = 0;
    
    pthread_mutex_unlock(&logger->mutex);
    pthread_mutex_destroy(&logger->mutex);
}

void logger_log(Logger_t *logger, LogLevel_t level, LogModule_t module,
                const char *file, int line, const char *fmt, ...) {
    if (logger == NULL || level > logger->level) {
        return;
    }
    
    /* 检查模块有效性 */
    if (module >= LOG_MODULE_NUM) {
        module = LOG_MODULE_SYS;
    }
    
    pthread_mutex_lock(&logger->mutex);
    
    /* 获取当前时间 */
    time_t now;
    struct tm *local;
    char time_str[32];
    
    time(&now);
    local = localtime(&now);
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", local);
    
    /* 提取文件名（不含路径） */
    const char *filename = strrchr(file, '/');
    if (filename == NULL) {
        filename = file;
    } else {
        filename++;  /* 跳过'/' */
    }
    
    /* 格式化日志消息 */
    char message[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);
    
    /* 输出到控制台 */
    if (logger->enable_console) {
        printf("%s[%s]%s [%s] [%s:%d] %s\n",
               level_colors[level],
               time_str,
               level_names[level],
               module_names[module],
               filename, line,
               message);
        printf("%s", color_reset);
        fflush(stdout);
    }
    
    /* 输出到文件 */
    if (logger->enable_file && logger->fp != NULL) {
        fprintf(logger->fp, "[%s] [%s] [%s] [%s:%d] %s\n",
                time_str,
                level_names[level],
                module_names[module],
                filename, line,
                message);
        fflush(logger->fp);
    }
    
    pthread_mutex_unlock(&logger->mutex);
}

void logger_set_level(Logger_t *logger, LogLevel_t level) {
    if (logger == NULL) {
        return;
    }
    
    pthread_mutex_lock(&logger->mutex);
    logger->level = level;
    pthread_mutex_unlock(&logger->mutex);
}

const char* logger_get_module_name(LogModule_t module) {
    if (module >= LOG_MODULE_NUM) {
        return "UNKNOWN";
    }
    return module_names[module];
}

const char* logger_get_level_name(LogLevel_t level) {
    if (level > LOG_LEVEL_TRACE) {
        return "UNKNOWN";
    }
    return level_names[level];
}
