/******************************************************************************
 * @file    gravity_unload.c
 * @brief   重力卸载控制算法实现
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#include "gravity_unload.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

/* 外部依赖声明 - 需要在主程序中提供 */
extern int get_sensor_data(SensorDataRaw_t *data);
extern int set_motor_velocity(float velocity);
extern int set_clutch_current(float current_mA);
extern int get_motor_actual_velocity(float *velocity);
extern uint32_t get_timestamp_ms(void);

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void* gravity_unload_thread(void *arg);
static void process_sensor_data(GravityUnloadController_t *ctrl, 
                                const SensorDataRaw_t *raw,
                                SensorDataFiltered_t *filtered);
static void calculate_control_output(GravityUnloadController_t *ctrl,
                                     const SensorDataFiltered_t *filtered,
                                     ControlOutput_t *output);

/******************************************************************************
 * 初始化与反初始化
 ******************************************************************************/

int gravity_unload_init(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return -1;
    
    memset(ctrl, 0, sizeof(GravityUnloadController_t));
    
    /* 初始化配置参数 */
    ctrl->pulley_r1_m = PULLEY_R1_MOTOR_RADIUS_M;
    ctrl->pulley_r2_m = PULLEY_R2_ENCODER_RADIUS_M;
    ctrl->clutch_current_per_torque = CLUTCH_CURRENT_PER_TORQUE_MA_NM;
    ctrl->motor_speed_compensation = MOTOR_SPEED_COMPENSATION_C;
    
    /* 初始化滤波器 */
    ma_filter_init(&ctrl->velocity_filter);
    lpf_init(&ctrl->pressure_filter, PRESSURE_FILTER_ALPHA);
    diff_init(&ctrl->pressure_diff, 0.5f);
    diff_init(&ctrl->position_diff, 0.5f);
    
    /* 初始化PID控制器 */
    pid_init(&ctrl->pid, PID_KP, PID_KI, PID_KD, 
             PID_OUTPUT_MIN, PID_OUTPUT_MAX, ALGO_CONTROL_PERIOD_S);
    ctrl->pid.integral_limit = PID_INTEGRAL_LIMIT;
    
    /* 初始化安全监控 */
    safety_monitor_init(&ctrl->safety);
    
    /* 初始化状态 */
    ctrl->status.state = ALGO_STATE_INIT;
    ctrl->status.error = ALGO_ERR_NONE;
    ctrl->status.cycle_count = 0;
    ctrl->status.error_count = 0;
    ctrl->status.running_time_s = 0.0f;
    ctrl->status.emergency_stop = 0;
    
    /* 初始化统计 */
    ctrl->max_pressure_kg = -9999.0f;
    ctrl->min_pressure_kg = 9999.0f;
    ctrl->max_velocity_m_s = 0.0f;
    
    /* 初始化线程控制 */
    ctrl->running = 0;
    ctrl->paused = 0;
    if (pthread_mutex_init(&ctrl->mutex, NULL) != 0) {
        return -1;
    }
    
    ctrl->first_run = 1;
    
    printf("[GRAVITY_UNLOAD] Controller initialized\n");
    printf("  Pulley R1: %.3f m, R2: %.3f m\n", ctrl->pulley_r1_m, ctrl->pulley_r2_m);
    printf("  Clutch current/torque: %.2f mA/Nm\n", ctrl->clutch_current_per_torque);
    printf("  Control period: %d ms\n", ALGO_CONTROL_PERIOD_MS);
    
    return 0;
}

void gravity_unload_deinit(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    gravity_unload_stop(ctrl);
    pthread_mutex_destroy(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Controller deinitialized\n");
}

/******************************************************************************
 * 线程控制
 ******************************************************************************/

int gravity_unload_start(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return -1;
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (ctrl->running) {
        pthread_mutex_unlock(&ctrl->mutex);
        return 0; /* 已经在运行 */
    }
    
    ctrl->running = 1;
    ctrl->paused = 0;
    ctrl->status.state = ALGO_STATE_RUNNING;
    
    if (pthread_create(&ctrl->thread_id, NULL, gravity_unload_thread, ctrl) != 0) {
        ctrl->running = 0;
        pthread_mutex_unlock(&ctrl->mutex);
        return -1;
    }
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Control thread started\n");
    return 0;
}

void gravity_unload_stop(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (!ctrl->running) {
        pthread_mutex_unlock(&ctrl->mutex);
        return;
    }
    
    ctrl->running = 0;
    ctrl->status.state = ALGO_STATE_SHUTDOWN;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    pthread_join(ctrl->thread_id, NULL);
    
    printf("[GRAVITY_UNLOAD] Control thread stopped\n");
}

void gravity_unload_pause(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->paused = 1;
    ctrl->status.state = ALGO_STATE_PAUSED;
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Control paused\n");
}

void gravity_unload_resume(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->paused = 0;
    ctrl->status.state = ALGO_STATE_RUNNING;
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Control resumed\n");
}

void gravity_unload_emergency_stop(GravityUnloadController_t *ctrl, const char *reason) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    
    ctrl->status.emergency_stop = 1;
    ctrl->status.state = ALGO_STATE_EMERGENCY_STOP;
    
    /* 触发安全监控的紧急停止 */
    safety_trigger_emergency_stop(&ctrl->safety, reason);
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] EMERGENCY STOP: %s\n", reason ? reason : "Unknown");
}

void gravity_unload_reset(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    
    /* 重置滤波器 */
    ma_filter_reset(&ctrl->velocity_filter);
    lpf_reset(&ctrl->pressure_filter);
    diff_reset(&ctrl->pressure_diff);
    diff_reset(&ctrl->position_diff);
    
    /* 重置PID */
    pid_reset(&ctrl->pid);
    
    /* 重置安全监控 */
    safety_clear_emergency_stop(&ctrl->safety);
    
    /* 重置状态 */
    ctrl->status.state = ALGO_STATE_INIT;
    ctrl->status.error = ALGO_ERR_NONE;
    ctrl->status.cycle_count = 0;
    ctrl->status.error_count = 0;
    ctrl->status.running_time_s = 0.0f;
    ctrl->status.emergency_stop = 0;
    
    ctrl->first_run = 1;
    ctrl->paused = 0;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Controller reset\n");
}

/******************************************************************************
 * 传感器数据处理
 ******************************************************************************/

static void process_sensor_data(GravityUnloadController_t *ctrl, 
                                const SensorDataRaw_t *raw,
                                SensorDataFiltered_t *filtered) {
    if (ctrl == NULL || raw == NULL || filtered == NULL) return;
    
    /* 压力低通滤波 */
    filtered->pressure_kg = lpf_update(&ctrl->pressure_filter, raw->pressure_kg);
    
    /* 压力微分（变化率） */
    filtered->pressure_derivative = diff_update(&ctrl->pressure_diff, 
                                                 filtered->pressure_kg, 
                                                 raw->timestamp_ms);
    
    /* 位置 */
    filtered->position_m = raw->encoder_position_m;
    
    /* 速度计算：位置微分 + 移动平均滤波 */
    float raw_velocity = diff_update(&ctrl->position_diff,
                                      filtered->position_m,
                                      raw->timestamp_ms);
    filtered->velocity_raw_m_s = raw_velocity;
    filtered->velocity_m_s = ma_filter_update(&ctrl->velocity_filter, raw_velocity);
    
    filtered->timestamp_ms = raw->timestamp_ms;
}

/******************************************************************************
 * 控制输出计算
 ******************************************************************************/

static void calculate_control_output(GravityUnloadController_t *ctrl,
                                     const SensorDataFiltered_t *filtered,
                                     ControlOutput_t *output) {
    if (ctrl == NULL || filtered == NULL || output == NULL) return;
    
    /* 步骤1: 计算离合器目标转矩 */
    /* 力矩 = 压力(kg) * g * R2 */
    /* 简化: 直接按比例计算电流 */
    float force_n = filtered->pressure_kg * 9.81f;  /* 转换为牛顿 */
    float torque_nm = force_n * ctrl->pulley_r2_m;   /* 力矩 = 力 * 半径 */
    
    /* 计算目标电流 */
    output->clutch_torque_nm = torque_nm;
    output->clutch_current_mA = torque_nm * ctrl->clutch_current_per_torque;
    
    /* 限制电流范围 */
    if (output->clutch_current_mA < SAFETY_CLUTCH_CURRENT_MIN_MA) {
        output->clutch_current_mA = SAFETY_CLUTCH_CURRENT_MIN_MA;
    } else if (output->clutch_current_mA > SAFETY_CLUTCH_CURRENT_MAX_MA) {
        output->clutch_current_mA = SAFETY_CLUTCH_CURRENT_MAX_MA;
    }
    
    /* 步骤2: 计算电机目标速度 */
    /* 电机角速度 = (V / R1 + C) * 编码器单位转换 */
    /* V是重物速度(m/s)，需要转换为电机速度指令 */
    float target_motor_speed = (filtered->velocity_m_s / ctrl->pulley_r1_m) * (1.0f + ctrl->motor_speed_compensation);
    
    /* 使用PID控制器 */
    float pid_output = pid_update(&ctrl->pid, target_motor_speed, output->motor_velocity_actual);
    
    output->motor_velocity_cmd = pid_output;
    output->timestamp_ms = filtered->timestamp_ms;
}

/******************************************************************************
 * 控制周期
 ******************************************************************************/

AlgoError_t gravity_unload_control_cycle(GravityUnloadController_t *ctrl,
                                          const SensorDataRaw_t *raw_data,
                                          SensorDataFiltered_t *filtered_data,
                                          ControlOutput_t *control_output) {
    if (ctrl == NULL || raw_data == NULL || filtered_data == NULL || control_output == NULL) {
        return ALGO_ERR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    
    /* 检查紧急停止 */
    if (ctrl->status.emergency_stop) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ALGO_ERR_SAFETY_VIOLATION;
    }
    
    /* 检查暂停 */
    if (ctrl->paused) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ALGO_ERR_NONE;
    }
    
    /* 更新统计 */
    if (raw_data->pressure_kg > ctrl->max_pressure_kg) {
        ctrl->max_pressure_kg = raw_data->pressure_kg;
    }
    if (raw_data->pressure_kg < ctrl->min_pressure_kg) {
        ctrl->min_pressure_kg = raw_data->pressure_kg;
    }
    
    /* 步骤1: 处理传感器数据 */
    process_sensor_data(ctrl, raw_data, filtered_data);
    
    /* 步骤2: 获取电机实际速度 */
    float actual_velocity = 0.0f;
    get_motor_actual_velocity(&actual_velocity);
    control_output->motor_velocity_actual = actual_velocity;
    
    /* 步骤3: 计算控制输出 */
    calculate_control_output(ctrl, filtered_data, control_output);
    
    /* 步骤4: 安全监控检查 */
    safety_monitor_update(&ctrl->safety, filtered_data, control_output, actual_velocity);
    SafetyStatus_t safety_status = safety_check(&ctrl->safety);
    
    if (safety_status == SAFETY_STATUS_EMERGENCY) {
        ctrl->status.state = ALGO_STATE_EMERGENCY_STOP;
        ctrl->status.error = ALGO_ERR_SAFETY_VIOLATION;
        ctrl->status.error_count++;
        pthread_mutex_unlock(&ctrl->mutex);
        return ALGO_ERR_SAFETY_VIOLATION;
    } else if (safety_status == SAFETY_STATUS_ERROR) {
        ctrl->status.error = ctrl->safety.error_code;
        ctrl->status.error_count++;
    }
    
    /* 更新统计 */
    if (fabsf(filtered_data->velocity_m_s) > ctrl->max_velocity_m_s) {
        ctrl->max_velocity_m_s = fabsf(filtered_data->velocity_m_s);
    }
    
    /* 更新时间 */
    if (ctrl->first_run) {
        ctrl->last_timestamp_ms = raw_data->timestamp_ms;
        ctrl->first_run = 0;
    } else {
        uint32_t dt_ms = raw_data->timestamp_ms - ctrl->last_timestamp_ms;
        ctrl->status.running_time_s += dt_ms / 1000.0f;
        ctrl->last_timestamp_ms = raw_data->timestamp_ms;
    }
    
    ctrl->status.cycle_count = ++ctrl->cycle_count;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    return ALGO_ERR_NONE;
}

/******************************************************************************
 * 线程函数
 ******************************************************************************/

static void* gravity_unload_thread(void *arg) {
    GravityUnloadController_t *ctrl = (GravityUnloadController_t *)arg;
    
    printf("[GRAVITY_UNLOAD] Thread started\n");
    
    SensorDataRaw_t raw_data;
    SensorDataFiltered_t filtered_data;
    ControlOutput_t control_output;
    
    while (1) {
        pthread_mutex_lock(&ctrl->mutex);
        int should_run = ctrl->running;
        pthread_mutex_unlock(&ctrl->mutex);
        
        if (!should_run) {
            break;
        }
        
        /* 获取传感器数据 */
        if (get_sensor_data(&raw_data) != 0) {
            printf("[GRAVITY_UNLOAD] Failed to get sensor data\n");
            usleep(ALGO_CONTROL_PERIOD_MS * 1000);
            continue;
        }
        
        /* 执行控制周期 */
        AlgoError_t err = gravity_unload_control_cycle(ctrl, &raw_data, &filtered_data, &control_output);
        
        if (err == ALGO_ERR_SAFETY_VIOLATION) {
            printf("[GRAVITY_UNLOAD] Safety violation detected!\n");
            /* 紧急停止：输出清零 */
            set_motor_velocity(0);
            set_clutch_current(0);
            break;
        }
        
        /* 输出到执行器 */
        set_motor_velocity(control_output.motor_velocity_cmd);
        set_clutch_current(control_output.clutch_current_mA);
        
        /* 周期控制 */
        usleep(ALGO_CONTROL_PERIOD_MS * 1000);
    }
    
    printf("[GRAVITY_UNLOAD] Thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 状态查询与调试
 ******************************************************************************/

void gravity_unload_get_status(const GravityUnloadController_t *ctrl, AlgoStatus_t *status) {
    if (ctrl == NULL || status == NULL) return;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    memcpy(status, &ctrl->status, sizeof(AlgoStatus_t));
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
}

void gravity_unload_print_status(const GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    
    printf("\n========== Gravity Unload Status ==========\n");
    printf("State: %d, Error: %d\n", ctrl->status.state, ctrl->status.error);
    printf("Cycle count: %u\n", ctrl->cycle_count);
    printf("Running time: %.1f s\n", ctrl->status.running_time_s);
    printf("Emergency stop: %s\n", ctrl->status.emergency_stop ? "YES" : "NO");
    printf("\nStatistics:\n");
    printf("  Pressure: %.2f to %.2f kg\n", ctrl->min_pressure_kg, ctrl->max_pressure_kg);
    printf("  Max velocity: %.3f m/s\n", ctrl->max_velocity_m_s);
    printf("\nSafety Status: %s\n", safety_status_to_string(ctrl->safety.status));
    if (ctrl->safety.error_msg[0] != '\0') {
        printf("  Message: %s\n", ctrl->safety.error_msg);
    }
    printf("==========================================\n");
    
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
}
