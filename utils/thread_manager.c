/******************************************************************************
 * @file    thread_manager.c
 * @brief   线程管理器实现 - 支持多线程管理
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.1.0
 ******************************************************************************/

#include "thread_manager.h"
#include "logger.h"

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>

/******************************************************************************
 * 线程包装参数
 ******************************************************************************/
typedef struct {
    ThreadCtrl_t *ctrl;
    ThreadManager_t *mgr;
} ThreadWrapperArg_t;

/******************************************************************************
 * 线程包装函数
 ******************************************************************************/
static void* thread_wrapper(void *arg) {
    ThreadWrapperArg_t *wrapper = (ThreadWrapperArg_t*)arg;
    ThreadCtrl_t *ctrl = wrapper->ctrl;
    ThreadManager_t *mgr = wrapper->mgr;
    
    /* 设置线程优先级 */
    if (ctrl->config.priority > 0) {
        thread_set_priority(ctrl->tid, ctrl->config.priority);
    }
    
    /* 更新状态 */
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->state = THREAD_STATE_INIT;
    pthread_mutex_unlock(&ctrl->mutex);
    
    LOG_INFO(LOG_MODULE_SYS, "Thread '%s' started", ctrl->config.name);
    
    /* 执行用户线程函数 */
    ctrl->config.func(ctrl);
    
    /* 线程结束 */
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->state = THREAD_STATE_STOPPED;
    pthread_mutex_unlock(&ctrl->mutex);
    
    LOG_INFO(LOG_MODULE_SYS, "Thread '%s' stopped", ctrl->config.name);
    
    free(wrapper);
    return NULL;
}

/******************************************************************************
 * 线程管理器接口实现
 ******************************************************************************/

ErrorCode_t thread_mgr_init(ThreadManager_t *mgr) {
    if (mgr == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(mgr, 0, sizeof(ThreadManager_t));
    
    if (pthread_mutex_init(&mgr->mutex, NULL) != 0) {
        return ERR_GENERAL;
    }
    
    /* 初始化所有线程控制块 */
    for (int i = 0; i < MAX_THREADS; i++) {
        pthread_mutex_init(&mgr->threads[i].mutex, NULL);
        mgr->threads[i].active = 0;
        mgr->threads[i].state = THREAD_STATE_STOPPED;
    }
    
    mgr->thread_count = 0;
    mgr->system_state = SYS_STATE_INIT;
    
    LOG_INFO(LOG_MODULE_SYS, "Thread manager initialized (max %d threads)", MAX_THREADS);
    return ERR_OK;
}

void thread_mgr_deinit(ThreadManager_t *mgr) {
    if (mgr == NULL) {
        return;
    }
    
    /* 停止所有线程 */
    thread_mgr_stop_all(mgr);
    
    /* 销毁所有互斥锁 */
    for (int i = 0; i < MAX_THREADS; i++) {
        pthread_mutex_destroy(&mgr->threads[i].mutex);
    }
    
    pthread_mutex_destroy(&mgr->mutex);
    
    LOG_INFO(LOG_MODULE_SYS, "Thread manager deinitialized");
}

ErrorCode_t thread_mgr_create(ThreadManager_t *mgr, const ThreadConfig_t *config) {
    if (mgr == NULL || config == NULL || config->func == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    if (mgr->thread_count >= MAX_THREADS) {
        LOG_ERROR(LOG_MODULE_SYS, "Maximum thread count (%d) reached", MAX_THREADS);
        return ERR_GENERAL;
    }
    
    pthread_mutex_lock(&mgr->mutex);
    
    /* 查找空闲槽位 */
    int slot = -1;
    for (int i = 0; i < MAX_THREADS; i++) {
        if (!mgr->threads[i].active) {
            slot = i;
            break;
        }
    }
    
    if (slot < 0) {
        pthread_mutex_unlock(&mgr->mutex);
        return ERR_GENERAL;
    }
    
    ThreadCtrl_t *ctrl = &mgr->threads[slot];
    
    pthread_mutex_lock(&ctrl->mutex);
    
    /* 复制配置 */
    memcpy(&ctrl->config, config, sizeof(ThreadConfig_t));
    ctrl->should_exit = 0;
    ctrl->state = THREAD_STATE_INIT;
    ctrl->active = 1;
    memset(&ctrl->stats, 0, sizeof(ThreadStats_t));
    
    /* 创建包装参数 */
    ThreadWrapperArg_t *wrapper = malloc(sizeof(ThreadWrapperArg_t));
    if (wrapper == NULL) {
        pthread_mutex_unlock(&ctrl->mutex);
        pthread_mutex_unlock(&mgr->mutex);
        return ERR_NO_MEMORY;
    }
    wrapper->ctrl = ctrl;
    wrapper->mgr = mgr;
    
    /* 创建线程 */
    if (pthread_create(&ctrl->tid, NULL, thread_wrapper, wrapper) != 0) {
        free(wrapper);
        ctrl->active = 0;
        pthread_mutex_unlock(&ctrl->mutex);
        pthread_mutex_unlock(&mgr->mutex);
        return ERR_GENERAL;
    }
    
    mgr->thread_count++;
    
    pthread_mutex_unlock(&ctrl->mutex);
    pthread_mutex_unlock(&mgr->mutex);
    
    LOG_INFO(LOG_MODULE_SYS, "Thread '%s' created (slot %d)", config->name, slot);
    return ERR_OK;
}

ErrorCode_t thread_mgr_stop_by_index(ThreadManager_t *mgr, int index) {
    if (mgr == NULL || index < 0 || index >= MAX_THREADS) {
        return ERR_INVALID_PARAM;
    }
    
    ThreadCtrl_t *ctrl = &mgr->threads[index];
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (!ctrl->active || ctrl->state == THREAD_STATE_STOPPED) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ERR_OK;
    }
    
    ctrl->should_exit = 1;
    ctrl->state = THREAD_STATE_STOPPING;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    /* 等待线程结束 */
    pthread_join(ctrl->tid, NULL);
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->active = 0;
    ctrl->state = THREAD_STATE_STOPPED;
    pthread_mutex_unlock(&ctrl->mutex);
    
    pthread_mutex_lock(&mgr->mutex);
    mgr->thread_count--;
    pthread_mutex_unlock(&mgr->mutex);
    
    LOG_INFO(LOG_MODULE_SYS, "Thread '%s' stopped", ctrl->config.name);
    return ERR_OK;
}

void thread_mgr_stop_all(ThreadManager_t *mgr) {
    if (mgr == NULL) {
        return;
    }
    
    LOG_INFO(LOG_MODULE_SYS, "Stopping all threads...");
    
    /* 先标记所有线程退出 */
    for (int i = 0; i < MAX_THREADS; i++) {
        ThreadCtrl_t *ctrl = &mgr->threads[i];
        pthread_mutex_lock(&ctrl->mutex);
        if (ctrl->active && ctrl->state != THREAD_STATE_STOPPED) {
            ctrl->should_exit = 1;
            ctrl->state = THREAD_STATE_STOPPING;
        }
        pthread_mutex_unlock(&ctrl->mutex);
    }
    
    /* 等待所有线程结束 */
    for (int i = 0; i < MAX_THREADS; i++) {
        ThreadCtrl_t *ctrl = &mgr->threads[i];
        pthread_mutex_lock(&ctrl->mutex);
        if (ctrl->active) {
            pthread_t tid = ctrl->tid;
            pthread_mutex_unlock(&ctrl->mutex);
            pthread_join(tid, NULL);
            
            pthread_mutex_lock(&ctrl->mutex);
            ctrl->active = 0;
            ctrl->state = THREAD_STATE_STOPPED;
            pthread_mutex_unlock(&ctrl->mutex);
            
            LOG_INFO(LOG_MODULE_SYS, "Thread '%s' stopped", ctrl->config.name);
        } else {
            pthread_mutex_unlock(&ctrl->mutex);
        }
    }
    
    pthread_mutex_lock(&mgr->mutex);
    mgr->thread_count = 0;
    pthread_mutex_unlock(&mgr->mutex);
}

ErrorCode_t thread_mgr_pause_by_index(ThreadManager_t *mgr, int index) {
    if (mgr == NULL || index < 0 || index >= MAX_THREADS) {
        return ERR_INVALID_PARAM;
    }
    
    ThreadCtrl_t *ctrl = &mgr->threads[index];
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (!ctrl->active || ctrl->state != THREAD_STATE_RUNNING) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ERR_INVALID_STATE;
    }
    
    ctrl->state = THREAD_STATE_PAUSED;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    return ERR_OK;
}

ErrorCode_t thread_mgr_resume_by_index(ThreadManager_t *mgr, int index) {
    if (mgr == NULL || index < 0 || index >= MAX_THREADS) {
        return ERR_INVALID_PARAM;
    }
    
    ThreadCtrl_t *ctrl = &mgr->threads[index];
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (!ctrl->active || ctrl->state != THREAD_STATE_PAUSED) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ERR_INVALID_STATE;
    }
    
    ctrl->state = THREAD_STATE_RUNNING;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    return ERR_OK;
}

ThreadState_t thread_mgr_get_state_by_index(ThreadManager_t *mgr, int index) {
    if (mgr == NULL || index < 0 || index >= MAX_THREADS) {
        return THREAD_STATE_ERROR;
    }
    
    ThreadCtrl_t *ctrl = &mgr->threads[index];
    ThreadState_t state;
    
    pthread_mutex_lock(&ctrl->mutex);
    state = ctrl->state;
    pthread_mutex_unlock(&ctrl->mutex);
    
    return state;
}

ErrorCode_t thread_mgr_get_stats_by_index(ThreadManager_t *mgr, int index, ThreadStats_t *stats) {
    if (mgr == NULL || index < 0 || index >= MAX_THREADS || stats == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    ThreadCtrl_t *ctrl = &mgr->threads[index];
    
    pthread_mutex_lock(&ctrl->mutex);
    memcpy(stats, &ctrl->stats, sizeof(ThreadStats_t));
    pthread_mutex_unlock(&ctrl->mutex);
    
    return ERR_OK;
}

void thread_mgr_print_status(ThreadManager_t *mgr) {
    if (mgr == NULL) {
        return;
    }
    
    LOG_INFO(LOG_MODULE_SYS, "Thread Status (total: %d):", mgr->thread_count);
    LOG_INFO(LOG_MODULE_SYS, "%-20s %-10s %-10s %-10s", "Name", "Type", "State", "Loops");
    LOG_INFO(LOG_MODULE_SYS, "------------------------------------------------");
    
    for (int i = 0; i < MAX_THREADS; i++) {
        ThreadCtrl_t *ctrl = &mgr->threads[i];
        pthread_mutex_lock(&ctrl->mutex);
        
        if (ctrl->active) {
            const char *state_str = "Unknown";
            switch (ctrl->state) {
                case THREAD_STATE_INIT:     state_str = "Init"; break;
                case THREAD_STATE_RUNNING:  state_str = "Running"; break;
                case THREAD_STATE_PAUSED:   state_str = "Paused"; break;
                case THREAD_STATE_STOPPING: state_str = "Stopping"; break;
                case THREAD_STATE_STOPPED:  state_str = "Stopped"; break;
                case THREAD_STATE_ERROR:    state_str = "Error"; break;
                default: break;
            }
            
            const char *type_str = "Unknown";
            switch (ctrl->config.type) {
                case THREAD_TYPE_CONTROL: type_str = "Control"; break;
                case THREAD_TYPE_DATA:    type_str = "Data"; break;
                case THREAD_TYPE_LOG:     type_str = "Log"; break;
                case THREAD_TYPE_COMM:    type_str = "Comm"; break;
                case THREAD_TYPE_ALGO:    type_str = "Algo"; break;
                case THREAD_TYPE_USER:    type_str = "User"; break;
                default: break;
            }
            
            LOG_INFO(LOG_MODULE_SYS, "%-20s %-10s %-10s %-10lu",
                     ctrl->config.name, type_str, state_str, ctrl->stats.loop_count);
        }
        
        pthread_mutex_unlock(&ctrl->mutex);
    }
}

/******************************************************************************
 * 线程工具函数
 ******************************************************************************/

ErrorCode_t thread_set_priority(pthread_t tid, int priority) {
    struct sched_param param;
    param.sched_priority = priority;
    
    if (pthread_setschedparam(tid, SCHED_FIFO, &param) != 0) {
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
        /* 简单移动平均 */
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
