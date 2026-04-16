/******************************************************************************
 * @file    thread_manager.c
 * @brief   线程管理器实现
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#include "thread_manager.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>

/* 线程名称映射 */
static const char *thread_names[THREAD_TYPE_NUM] = {
    [THREAD_TYPE_CONTROL] = "Control",
    [THREAD_TYPE_DATA]    = "DataAcq",
    [THREAD_TYPE_LOG]     = "Logger",
    [THREAD_TYPE_COMM]    = "Comm",
    [THREAD_TYPE_ALGO]    = "Algo"
};

ErrorCode_t thread_mgr_init(ThreadManager_t *mgr) {
    if (mgr == NULL) {
        return ERR_INVALID_PARAM;
    }

    memset(mgr, 0, sizeof(ThreadManager_t));
    
    if (pthread_mutex_init(&mgr->mutex, NULL) != 0) {
        return ERR_GENERAL;
    }

    mgr->system_state = SYS_STATE_INIT;
    mgr->thread_count = 0;

    for (int i = 0; i < THREAD_TYPE_NUM; i++) {
        mgr->threads[i].type = i;
        mgr->threads[i].state = THREAD_STATE_INIT;
        mgr->threads[i].should_exit = 0;
        pthread_mutex_init(&mgr->threads[i].mutex, NULL);
    }

    LOG_INFO(LOG_MODULE_SYS, "Thread manager initialized");
    return ERR_OK;
}

void thread_mgr_deinit(ThreadManager_t *mgr) {
    if (mgr == NULL) {
        return;
    }

    thread_mgr_stop_all(mgr);

    pthread_mutex_destroy(&mgr->mutex);
    
    for (int i = 0; i < THREAD_TYPE_NUM; i++) {
        pthread_mutex_destroy(&mgr->threads[i].mutex);
    }

    LOG_INFO(LOG_MODULE_SYS, "Thread manager deinitialized");
}

typedef struct {
    ThreadCtrl_t *ctrl;
    ThreadManager_t *mgr;
} ThreadWrapperArg_t;

static void* thread_wrapper(void *arg) {
    ThreadWrapperArg_t *wrapper = (ThreadWrapperArg_t*)arg;
    ThreadCtrl_t *ctrl = wrapper->ctrl;
    ThreadConfig_t *config = &ctrl->config;
    
    /* 设置线程优先级 */
    if (config->priority > 0) {
        thread_set_priority(pthread_self(), config->priority);
    }

    /* 调用实际线程函数 */
    void *result = config->func(ctrl);
    
    free(wrapper);
    return result;
}

ErrorCode_t thread_mgr_create(ThreadManager_t *mgr, const ThreadConfig_t *config) {
    if (mgr == NULL || config == NULL || config->type >= THREAD_TYPE_NUM) {
        return ERR_INVALID_PARAM;
    }

    ThreadCtrl_t *ctrl = &mgr->threads[config->type];
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (ctrl->state != THREAD_STATE_INIT && ctrl->state != THREAD_STATE_STOPPED) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ERR_ALREADY_INIT;
    }

    /* 复制配置 */
    memcpy(&ctrl->config, config, sizeof(ThreadConfig_t));
    ctrl->should_exit = 0;
    ctrl->state = THREAD_STATE_INIT;
    memset(&ctrl->stats, 0, sizeof(ThreadStats_t));

    /* 创建包装参数 */
    ThreadWrapperArg_t *wrapper = malloc(sizeof(ThreadWrapperArg_t));
    if (wrapper == NULL) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ERR_NO_MEMORY;
    }
    wrapper->ctrl = ctrl;
    wrapper->mgr = mgr;

    /* 创建线程 */
    if (pthread_create(&ctrl->tid, NULL, thread_wrapper, wrapper) != 0) {
        free(wrapper);
        pthread_mutex_unlock(&ctrl->mutex);
        return ERR_GENERAL;
    }

    mgr->thread_count++;
    
    pthread_mutex_unlock(&ctrl->mutex);

    LOG_INFO(LOG_MODULE_SYS, "Thread '%s' created", config->name);
    return ERR_OK;
}

ErrorCode_t thread_mgr_stop(ThreadManager_t *mgr, ThreadType_t type) {
    if (mgr == NULL || type >= THREAD_TYPE_NUM) {
        return ERR_INVALID_PARAM;
    }

    ThreadCtrl_t *ctrl = &mgr->threads[type];
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (ctrl->state == THREAD_STATE_STOPPED || ctrl->state == THREAD_STATE_INIT) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ERR_OK;
    }

    ctrl->should_exit = 1;
    ctrl->state = THREAD_STATE_STOPPING;
    
    pthread_mutex_unlock(&ctrl->mutex);

    /* 等待线程结束 */
    pthread_join(ctrl->tid, NULL);
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->state = THREAD_STATE_STOPPED;
    pthread_mutex_unlock(&ctrl->mutex);

    mgr->thread_count--;
    
    LOG_INFO(LOG_MODULE_SYS, "Thread '%s' stopped", ctrl->config.name);
    return ERR_OK;
}

void thread_mgr_stop_all(ThreadManager_t *mgr) {
    if (mgr == NULL) {
        return;
    }

    for (int i = 0; i < THREAD_TYPE_NUM; i++) {
        thread_mgr_stop(mgr, i);
    }
}

ThreadState_t thread_mgr_get_state(ThreadManager_t *mgr, ThreadType_t type) {
    if (mgr == NULL || type >= THREAD_TYPE_NUM) {
        return THREAD_STATE_ERROR;
    }

    ThreadState_t state;
    pthread_mutex_lock(&mgr->threads[type].mutex);
    state = mgr->threads[type].state;
    pthread_mutex_unlock(&mgr->threads[type].mutex);
    
    return state;
}

ErrorCode_t thread_mgr_get_stats(ThreadManager_t *mgr, ThreadType_t type, ThreadStats_t *stats) {
    if (mgr == NULL || type >= THREAD_TYPE_NUM || stats == NULL) {
        return ERR_INVALID_PARAM;
    }

    pthread_mutex_lock(&mgr->threads[type].mutex);
    memcpy(stats, &mgr->threads[type].stats, sizeof(ThreadStats_t));
    pthread_mutex_unlock(&mgr->threads[type].mutex);
    
    return ERR_OK;
}

void thread_mgr_print_status(ThreadManager_t *mgr) {
    if (mgr == NULL) {
        return;
    }

    printf("\n--- Thread Status ---\n");
    for (int i = 0; i < THREAD_TYPE_NUM; i++) {
        ThreadCtrl_t *ctrl = &mgr->threads[i];
        if (ctrl->state != THREAD_STATE_INIT) {
            const char *state_str;
            switch (ctrl->state) {
                case THREAD_STATE_INIT:     state_str = "INIT"; break;
                case THREAD_STATE_RUNNING:  state_str = "RUNNING"; break;
                case THREAD_STATE_PAUSED:   state_str = "PAUSED"; break;
                case THREAD_STATE_STOPPING: state_str = "STOPPING"; break;
                case THREAD_STATE_STOPPED:  state_str = "STOPPED"; break;
                case THREAD_STATE_ERROR:    state_str = "ERROR"; break;
                default:                    state_str = "UNKNOWN"; break;
            }
            printf("  %s: %s (loops: %lu)\n", 
                   ctrl->config.name ? ctrl->config.name : thread_names[i],
                   state_str, ctrl->stats.loop_count);
        }
    }
    printf("---------------------\n");
}

ErrorCode_t thread_set_priority(pthread_t tid, int priority) {
    struct sched_param param;
    param.sched_priority = priority;
    
    if (pthread_setschedparam(tid, SCHED_FIFO, &param) != 0) {
        /* 如果实时调度失败，尝试使用普通调度 */
        return ERR_GENERAL;
    }
    
    return ERR_OK;
}

int thread_should_exit(ThreadCtrl_t *ctrl) {
    if (ctrl == NULL) {
        return 1;
    }
    
    int should_exit;
    pthread_mutex_lock(&ctrl->mutex);
    should_exit = ctrl->should_exit;
    pthread_mutex_unlock(&ctrl->mutex);
    
    return should_exit;
}

void thread_update_stats(ThreadCtrl_t *ctrl, double cycle_time_ms) {
    if (ctrl == NULL) {
        return;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    
    ctrl->stats.loop_count++;
    
    /* 更新平均周期时间 */
    if (ctrl->stats.loop_count == 1) {
        ctrl->stats.avg_cycle_time_ms = cycle_time_ms;
        ctrl->stats.max_cycle_time_ms = cycle_time_ms;
        ctrl->stats.min_cycle_time_ms = cycle_time_ms;
    } else {
        ctrl->stats.avg_cycle_time_ms = 
            (ctrl->stats.avg_cycle_time_ms * (ctrl->stats.loop_count - 1) + cycle_time_ms) 
            / ctrl->stats.loop_count;
        
        if (cycle_time_ms > ctrl->stats.max_cycle_time_ms) {
            ctrl->stats.max_cycle_time_ms = cycle_time_ms;
        }
        if (cycle_time_ms < ctrl->stats.min_cycle_time_ms) {
            ctrl->stats.min_cycle_time_ms = cycle_time_ms;
        }
    }
    
    pthread_mutex_unlock(&ctrl->mutex);
}
