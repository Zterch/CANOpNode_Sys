/******************************************************************************
 * @file    motor_driver.h
 * @brief   CANopen电机驱动 - Nimotion伺服电机控制
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 电机状态定义
 ******************************************************************************/
typedef enum {
    MOTOR_STATE_INIT = 0,       /* 初始化 */
    MOTOR_STATE_NOT_READY,      /* 未就绪 */
    MOTOR_STATE_READY,          /* 就绪 */
    MOTOR_STATE_ENABLED,        /* 已使能 */
    MOTOR_STATE_FAULT,          /* 故障 */
    MOTOR_STATE_UNKNOWN         /* 未知 */
} MotorState_t;

/******************************************************************************
 * 电机工作模式
 ******************************************************************************/
typedef enum {
    MOTOR_MODE_PP = 1,          /* 轮廓位置模式 */
    MOTOR_MODE_VM = 2,          /* 速度模式 */
    MOTOR_MODE_PV = 3,          /* 轮廓速度模式 */
    MOTOR_MODE_PT = 4,          /* 轮廓转矩模式 */
    MOTOR_MODE_HM = 6,          /* 原点回归模式 */
    MOTOR_MODE_IP = 7,          /* 位置插补模式 */
    MOTOR_MODE_CSP = 8,         /* 循环同步位置模式 */
    MOTOR_MODE_CSV = 9,         /* 循环同步速度模式 */
    MOTOR_MODE_CST = 10         /* 循环同步转矩模式 */
} MotorMode_t;

/******************************************************************************
 * 电机数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    uint8_t node_id;            /* CAN节点ID */
    char can_interface[16];     /* CAN接口名称 */
    int socket_fd;              /* CAN套接字 */
    
    /* 状态信息 */
    MotorState_t state;         /* 电机状态 */
    MotorMode_t mode;           /* 当前模式 */
    uint16_t status_word;       /* 状态字 */
    
    /* 实时数据 */
    int32_t actual_position;    /* 实际位置 */
    int32_t actual_velocity;    /* 实际速度 */
    int32_t actual_torque;      /* 实际转矩 */
    int32_t target_velocity;    /* 目标速度 */
    int32_t target_position;    /* 目标位置 */
    
    /* 控制参数 */
    int32_t max_velocity;       /* 最大速度 */
    int32_t max_acceleration;   /* 最大加速度 */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    int initialized;
    int enabled;
} MotorDriver_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化电机驱动
 * @param motor 电机驱动结构指针
 * @param node_id CAN节点ID
 * @param can_if CAN接口名称
 * @return ErrorCode_t
 */
ErrorCode_t motor_init(MotorDriver_t *motor, uint8_t node_id, const char *can_if);

/**
 * @brief 反初始化电机驱动
 * @param motor 电机驱动结构指针
 */
void motor_deinit(MotorDriver_t *motor);

/**
 * @brief 使能电机
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_enable(MotorDriver_t *motor);

/**
 * @brief 失能电机
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_disable(MotorDriver_t *motor);

/**
 * @brief 设置工作模式
 * @param motor 电机驱动结构指针
 * @param mode 工作模式
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_mode(MotorDriver_t *motor, MotorMode_t mode);

/**
 * @brief 设置目标速度
 * @param motor 电机驱动结构指针
 * @param velocity 目标速度
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_velocity(MotorDriver_t *motor, int32_t velocity);

/**
 * @brief 设置目标位置
 * @param motor 电机驱动结构指针
 * @param position 目标位置
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_position(MotorDriver_t *motor, int32_t position);

/**
 * @brief 读取实际速度
 * @param motor 电机驱动结构指针
 * @param velocity 速度输出指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_get_velocity(MotorDriver_t *motor, int32_t *velocity);

/**
 * @brief 读取实际位置
 * @param motor 电机驱动结构指针
 * @param position 位置输出指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_get_position(MotorDriver_t *motor, int32_t *position);

/**
 * @brief 更新电机状态（周期性调用）
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_update_state(MotorDriver_t *motor);

/**
 * @brief 清除故障
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_clear_fault(MotorDriver_t *motor);

/**
 * @brief 获取电机状态字符串
 * @param state 电机状态
 * @return 状态字符串
 */
const char* motor_get_state_string(MotorState_t state);

/**
 * @brief 获取模式字符串
 * @param mode 工作模式
 * @return 模式字符串
 */
const char* motor_get_mode_string(MotorMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H__ */
