/******************************************************************************
 * @file    thread_manager.h
 * @brief   线程管理器 - 统一管理所有系统线程
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.1.0
 ******************************************************************************/

#ifndef __THREAD_MANAGER_H__
#define __THREAD_MANAGER_H__

#include <pthread.h>
#include <stdint.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 线程类型定义
 ******************************************************************************/
typedef enum {
    THREAD_TYPE_CONTROL = 0,    /* 控制线程 - 最高优先级 */
    THREAD_TYPE_DATA,           /* 数据采集线程 */
    THREAD_TYPE_LOG,            /* 日志线程 */
    THREAD_TYPE_COMM,           /* 通信线程 */
    THREAD_TYPE_ALGO,           /* 算法线程 */
    THREAD_TYPE_USER,           /* 用户自定义线程 */
    THREAD_TYPE_NUM             /* 线程类型数量 */
} ThreadType_t;

/******************************************************************************
 * 线程状态定义
 ******************************************************************************/
typedef enum {
    THREAD_STATE_INIT = 0,      /* 初始化 */
    THREAD_STATE_RUNNING,       /* 运行中 */
    THREAD_STATE_PAUSED,        /* 暂停 */
    THREAD_STATE_STOPPING,      /* 停止中 */
    THREAD_STATE_STOPPED,       /* 已停止 */
    THREAD_STATE_ERROR          /* 错误 */
} ThreadState_t;

/******************************************************************************
 * 线程统计信息
 ******************************************************************************/
typedef struct {
    uint64_t loop_count;        /* 循环计数 */
    uint64_t error_count;       /* 错误计数 */
    double avg_cycle_time_ms;   /* 平均周期时间 */
    double max_cycle_time_ms;   /* 最大周期时间 */
    double min_cycle_time_ms;   /* 最小周期时间 */
} ThreadStats_t;

/******************************************************************************
 * 线程配置结构
 ******************************************************************************/
typedef struct {
    ThreadType_t type;          /* 线程类型 */
    const char *name;           /* 线程名称 */
    int priority;               /* 线程优先级 */
    int period_ms;              /* 执行周期 (ms) */
    void *(*func)(void *);      /* 线程函数 */
    void *arg;                  /* 线程参数 */
} ThreadConfig_t;

/******************************************************************************
 * 线程控制块
 ******************************************************************************/
typedef struct {
    pthread_t tid;              /* 线程ID */
    ThreadType_t type;          /* 线程类型 */
    ThreadState_t state;        /* 线程状态 */
    ThreadStats_t stats;        /* 统计信息 */
    ThreadConfig_t config;      /* 配置信息 */
    pthread_mutex_t mutex;      /* 状态互斥锁 */
    int should_exit;            /* 退出标志 */
    int active;                 /* 是否激活 */
} ThreadCtrl_t;

/******************************************************************************
 * 线程管理器 - 支持最多16个线程
 ******************************************************************************/
#define MAX_THREADS 16

typedef struct {
    ThreadCtrl_t threads[MAX_THREADS];
    int thread_count;
    pthread_mutex_t mutex;
    SystemState_t system_state;
} ThreadManager_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化线程管理器
 * @param mgr 线程管理器指针
 * @return ErrorCode_t
 */
ErrorCode_t thread_mgr_init(ThreadManager_t *mgr);

/**
 * @brief 反初始化线程管理器
 * @param mgr 线程管理器指针
 */
void thread_mgr_deinit(ThreadManager_t *mgr);

/**
 * @brief 创建并启动线程
 * @param mgr 线程管理器指针
 * @param config 线程配置
 * @return ErrorCode_t
 */
ErrorCode_t thread_mgr_create(ThreadManager_t *mgr, const ThreadConfig_t *config);

/**
 * @brief 停止指定索引的线程
 * @param mgr 线程管理器指针
 * @param index 线程索引
 * @return ErrorCode_t
 */
ErrorCode_t thread_mgr_stop_by_index(ThreadManager_t *mgr, int index);

/**
 * @brief 停止所有线程
 * @param mgr 线程管理器指针
 */
void thread_mgr_stop_all(ThreadManager_t *mgr);

/**
 * @brief 暂停指定索引的线程
 * @param mgr 线程管理器指针
 * @param index 线程索引
 * @return ErrorCode_t
 */
ErrorCode_t thread_mgr_pause_by_index(ThreadManager_t *mgr, int index);

/**
 * @brief 恢复指定索引的线程
 * @param mgr 线程管理器指针
 * @param index 线程索引
 * @return ErrorCode_t
 */
ErrorCode_t thread_mgr_resume_by_index(ThreadManager_t *mgr, int index);

/**
 * @brief 获取线程状态
 * @param mgr 线程管理器指针
 * @param index 线程索引
 * @return ThreadState_t
 */
ThreadState_t thread_mgr_get_state_by_index(ThreadManager_t *mgr, int index);

/**
 * @brief 获取线程统计信息
 * @param mgr 线程管理器指针
 * @param index 线程索引
 * @param stats 统计信息输出
 * @return ErrorCode_t
 */
ErrorCode_t thread_mgr_get_stats_by_index(ThreadManager_t *mgr, int index, ThreadStats_t *stats);

/**
 * @brief 打印所有线程状态
 * @param mgr 线程管理器指针
 */
void thread_mgr_print_status(ThreadManager_t *mgr);

/**
 * @brief 设置线程优先级（实时调度）
 * @param tid 线程ID
 * @param priority 优先级 (1-99)
 * @return ErrorCode_t
 */
ErrorCode_t thread_set_priority(pthread_t tid, int priority);

/**
 * @brief 检查线程是否应该退出
 * @param ctrl 线程控制块
 * @return 1=应该退出, 0=继续运行
 */
int thread_should_exit(ThreadCtrl_t *ctrl);

/**
 * @brief 更新线程统计信息
 * @param ctrl 线程控制块
 * @param cycle_time_ms 本次周期时间
 */
void thread_update_stats(ThreadCtrl_t *ctrl, double cycle_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* __THREAD_MANAGER_H__ */
