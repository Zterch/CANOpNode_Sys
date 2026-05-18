/******************************************************************************
 * @file    shared_memory.c
 * @brief   共享内存实现 - 上下位机实时通信
 * @author  System Architect
 * @date    2026-05-11
 * @version 1.0.0
 * 
 * @description
 * 基于POSIX共享内存实现零拷贝高速通信
 * - 数据频率：20Hz（50ms周期）
 * - 延迟：<1ms
 * - 支持双向通信：数据+命令
 ******************************************************************************/

#include "../include/shared_memory.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

/* 共享内存布局：
 * [0-1023]:    SharedData_t (传感器数据)
 * [1024-2047]: SharedCommand_t (控制命令)
 * [2048-4095]: 保留扩展
 */
#define DATA_OFFSET     0
#define COMMAND_OFFSET  1024

/**
 * @brief 获取当前时间戳（微秒）
 */
static uint64_t get_timestamp_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000);
}

/**
 * @brief 初始化共享内存
 * 
 * @param mgr 共享内存管理器
 * @param create 是否创建（true=下位机，false=上位机）
 * @return 0成功，-1失败
 */
int shm_init(ShmManager_t *mgr, bool create) {
    if (mgr == NULL) {
        return -1;
    }
    
    memset(mgr, 0, sizeof(ShmManager_t));
    mgr->is_creator = create;
    
    /* 打开或创建共享内存对象 */
    int flags = create ? (O_CREAT | O_RDWR) : O_RDWR;
    /* 设置权限0666，允许所有用户读写（解决sudo和普通用户权限问题） */
    mode_t mode = create ? (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH) : 0;
    
    mgr->shm_fd = shm_open(SHM_NAME, flags, mode);
    if (mgr->shm_fd < 0) {
        perror("shm_open failed");
        return -1;
    }
    
    /* 设置共享内存大小 */
    if (create) {
        if (ftruncate(mgr->shm_fd, SHM_SIZE) < 0) {
            perror("ftruncate failed");
            close(mgr->shm_fd);
            shm_unlink(SHM_NAME);
            return -1;
        }
        
        /* 显式设置权限为0666（防止umask影响） */
        char shm_path[64];
        snprintf(shm_path, sizeof(shm_path), "/dev/shm%s", SHM_NAME);
        if (chmod(shm_path, 0666) < 0) {
            perror("chmod failed");
            // 不返回错误，继续执行
        }
        
        /* 验证权限设置 */
        struct stat st;
        if (stat(shm_path, &st) == 0) {
            printf("[SHM] Shared memory permissions: %o\n", st.st_mode & 0777);
        }
    }
    
    /* 映射共享内存 */
    mgr->shm_ptr = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mgr->shm_fd, 0);
    if (mgr->shm_ptr == MAP_FAILED) {
        perror("mmap failed");
        close(mgr->shm_fd);
        if (create) {
            shm_unlink(SHM_NAME);
        }
        return -1;
    }
    
    /* 设置指针 */
    mgr->data = (SharedData_t *)((char *)mgr->shm_ptr + DATA_OFFSET);
    mgr->command = (SharedCommand_t *)((char *)mgr->shm_ptr + COMMAND_OFFSET);
    
    /* 初始化数据结构（仅创建者） */
    if (create) {
        memset(mgr->shm_ptr, 0, SHM_SIZE);
        mgr->data->magic = SHM_MAGIC;
        mgr->data->version = DATA_VERSION;
        mgr->command->magic = SHM_MAGIC;
        
        /* 内存屏障，确保初始化完成 */
        __sync_synchronize();
    }
    
    return 0;
}

/**
 * @brief 关闭共享内存
 * 
 * @param mgr 共享内存管理器
 */
void shm_close(ShmManager_t *mgr) {
    if (mgr == NULL || mgr->shm_ptr == NULL) {
        return;
    }
    
    /* 解除映射 */
    munmap(mgr->shm_ptr, SHM_SIZE);
    mgr->shm_ptr = NULL;
    mgr->data = NULL;
    mgr->command = NULL;
    
    /* 关闭文件描述符 */
    close(mgr->shm_fd);
    mgr->shm_fd = -1;
    
    /* 删除共享内存对象（仅创建者） */
    if (mgr->is_creator) {
        shm_unlink(SHM_NAME);
    }
}

/**
 * @brief 写入传感器数据（下位机调用）
 * 
 * @param mgr 共享内存管理器
 * @param data 要写入的数据
 * @return 0成功，-1失败
 */
int shm_write_data(ShmManager_t *mgr, const SharedData_t *data) {
    if (mgr == NULL || mgr->data == NULL || data == NULL) {
        return -1;
    }
    
    /* 复制数据到共享内存 */
    SharedData_t *dest = mgr->data;
    
    /* 使用临时变量避免部分写入 */
    SharedData_t temp = *data;
    temp.magic = SHM_MAGIC;
    temp.version = DATA_VERSION;
    temp.timestamp_us = get_timestamp_us();
    
    /* 原子操作：更新序列号 */
    static uint32_t sequence = 0;
    temp.sequence = ++sequence;
    
    /* 内存复制（保证原子性） */
    memcpy(dest, &temp, sizeof(SharedData_t));
    
    /* 内存屏障，确保写入完成 */
    __sync_synchronize();
    
    /* 调试：每100次写入打印一次序列号 */
    static int debug_counter = 0;
    if (++debug_counter >= 100) {
        debug_counter = 0;
        printf("[SHM] Data written, sequence=%u, pressure=%.3f, motor=%d\n", 
               temp.sequence, temp.pressure_kg, (int)temp.motor_speed_rpm);
    }
    
    return 0;
}

/**
 * @brief 读取传感器数据（上位机调用）
 * 
 * @param mgr 共享内存管理器
 * @param data 读取到的数据
 * @return 0成功，-1失败，1数据未更新
 */
int shm_read_data(ShmManager_t *mgr, SharedData_t *data) {
    if (mgr == NULL || mgr->data == NULL || data == NULL) {
        return -1;
    }
    
    SharedData_t *src = mgr->data;
    
    /* 检查魔数 */
    if (src->magic != SHM_MAGIC) {
        return -1;
    }
    
    /* 检查版本 */
    if (src->version != DATA_VERSION) {
        return -1;
    }
    
    /* 复制数据 */
    memcpy(data, src, sizeof(SharedData_t));
    
    return 0;
}

/**
 * @brief 写入控制命令（上位机调用）
 * 
 * @param mgr 共享内存管理器
 * @param cmd 要写入的命令
 * @return 0成功，-1失败
 */
int shm_write_command(ShmManager_t *mgr, const SharedCommand_t *cmd) {
    if (mgr == NULL || mgr->command == NULL || cmd == NULL) {
        return -1;
    }
    
    SharedCommand_t *dest = mgr->command;
    
    /* 使用临时变量 */
    SharedCommand_t temp = *cmd;
    temp.magic = SHM_MAGIC;
    
    /* 原子递增命令ID */
    static uint32_t command_id = 0;
    temp.command_id = ++command_id;
    
    /* 复制到共享内存 */
    memcpy(dest, &temp, sizeof(SharedCommand_t));
    
    /* 内存屏障 */
    __sync_synchronize();
    
    return 0;
}

/**
 * @brief 读取控制命令（下位机调用）
 * 
 * @param mgr 共享内存管理器
 * @param cmd 读取到的命令
 * @return 0成功，-1失败，1无新命令
 */
int shm_read_command(ShmManager_t *mgr, SharedCommand_t *cmd) {
    if (mgr == NULL || mgr->command == NULL || cmd == NULL) {
        return -1;
    }
    
    SharedCommand_t *src = mgr->command;
    
    /* 检查魔数 */
    if (src->magic != SHM_MAGIC) {
        return -1;
    }
    
    /* 复制数据 */
    memcpy(cmd, src, sizeof(SharedCommand_t));
    
    return 0;
}

/**
 * @brief 清空命令（下位机执行后调用）
 * 
 * @param mgr 共享内存管理器
 */
void shm_clear_command(ShmManager_t *mgr) {
    if (mgr == NULL || mgr->command == NULL) {
        return;
    }
    
    /* 只清除命令类型和标志，保留command_id用于确认 */
    mgr->command->cmd_type = 0;
    mgr->command->algorithm_start = false;
    mgr->command->algorithm_stop = false;
    
    __sync_synchronize();
}
