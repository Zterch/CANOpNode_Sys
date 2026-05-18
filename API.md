# CANOpNode_Sys API 文档 v4.0

## 系统架构

基于NiMotion SDK的工业级CANopen控制系统

## 电机驱动API (motor_driver.h)

### 初始化与反初始化

```c
// 初始化电机驱动和SDK
ErrorCode_t motor_init(MotorDriver_t *motor, uint8_t node_id, const char *can_if);

// 反初始化电机驱动和SDK
void motor_deinit(MotorDriver_t *motor);
```

### 电机控制

```c
// 使能电机 (CSV模式)
ErrorCode_t motor_enable(MotorDriver_t *motor);

// 失能电机
ErrorCode_t motor_disable(MotorDriver_t *motor);

// 设置工作模式
ErrorCode_t motor_set_mode(MotorDriver_t *motor, MotorMode_t mode);

// 快速停止
ErrorCode_t motor_fast_stop(MotorDriver_t *motor);

// 清除故障
ErrorCode_t motor_clear_fault(MotorDriver_t *motor);
```

### 实时控制 (100Hz PDO)

```c
// 设置目标速度 (rpm) - PDO方式, 100Hz实时
ErrorCode_t motor_set_velocity(MotorDriver_t *motor, int32_t velocity);

// 设置目标位置 - PDO方式
ErrorCode_t motor_set_position(MotorDriver_t *motor, int32_t position);

// 读取实际速度 (rpm)
ErrorCode_t motor_get_velocity(MotorDriver_t *motor, int32_t *velocity);

// 读取实际位置
ErrorCode_t motor_get_position(MotorDriver_t *motor, int32_t *position);

// 获取电机实际位置（米）
float motor_get_position_m(MotorDriver_t *motor);

// 获取电机实际速度（rpm）
int32_t motor_get_velocity_rpm(MotorDriver_t *motor);
```

### 状态更新 (100Hz周期性调用)

```c
// 更新电机状态 (通过PDO读取)
ErrorCode_t motor_update_state(MotorDriver_t *motor);

// 获取状态字符串
const char* motor_get_state_string(MotorState_t state);

// 获取模式字符串
const char* motor_get_mode_string(MotorMode_t mode);

// 获取统计信息
void motor_get_statistics(MotorDriver_t *motor, uint64_t *tx_count, 
                          uint64_t *rx_count, uint64_t *err_count);
```

## 传感器管理API (sensor_manager.h)

### 初始化与配置

```c
// 初始化传感器管理器
ErrorCode_t sensor_mgr_init(SensorManager_t *mgr, const char *device, int baudrate);

// 反初始化
void sensor_mgr_deinit(SensorManager_t *mgr);

// 设置编码器绳索参数
void sensor_mgr_set_encoder_rope_params(SensorManager_t *mgr, 
                                        float wheel_circumference_mm, 
                                        int encoder_resolution);
```

### 数据采集

```c
// 启动传感器管理线程
ErrorCode_t sensor_mgr_start(SensorManager_t *mgr);

// 停止传感器管理线程
void sensor_mgr_stop(SensorManager_t *mgr);

// 获取传感器数据
ErrorCode_t sensor_mgr_get_data(SensorManager_t *mgr, 
                                SensorType_t type, 
                                SensorData_t *data);
```

### 校准

```c
// 编码器零点校准
void sensor_mgr_encoder_zero_calibration(SensorManager_t *mgr);

// 压力传感器去皮
void sensor_mgr_pressure_tare(SensorManager_t *mgr);
```

## 电源控制API (power_driver.h)

```c
// 初始化电源驱动
ErrorCode_t power_init(PowerDriver_t *power, const char *device, int baudrate);

// 反初始化
void power_deinit(PowerDriver_t *power);

// 设置电流 (mA)
ErrorCode_t power_set_current(PowerDriver_t *power, uint16_t current_ma);

// 获取状态
ErrorCode_t power_get_status(PowerDriver_t *power, uint16_t *current, uint16_t *voltage);
```

## 共享内存API (shared_memory.h)

```c
// 初始化共享内存
int shm_init(ShmManager_t *mgr, bool create);

// 关闭共享内存
void shm_close(ShmManager_t *mgr);

// 写入数据
int shm_write_data(ShmManager_t *mgr, const SharedData_t *data);

// 读取命令
int shm_read_command(ShmManager_t *mgr, SharedCommand_t *cmd);

// 清空命令
void shm_clear_command(ShmManager_t *mgr);
```

## 重力卸载算法API (gravity_unload.h)

```c
// 初始化算法
int gravity_unload_init(GravityUnloadController_t *ctrl);

// 启动算法
int gravity_unload_start(GravityUnloadController_t *ctrl);

// 停止算法
void gravity_unload_stop(GravityUnloadController_t *ctrl);

// 反初始化
void gravity_unload_deinit(GravityUnloadController_t *ctrl);

// 获取状态
void gravity_unload_get_status(GravityUnloadController_t *ctrl, AlgoStatus_t *status);
```

## 使用示例

### 电机控制示例

```c
#include "drivers/motor_driver.h"

MotorDriver_t motor;

// 初始化
motor_init(&motor, 1, "can0");

// 使能电机 (CSV模式)
motor_enable(&motor);

// 设置速度 1000 rpm
motor_set_velocity(&motor, 1000);

// 读取实际速度
int32_t actual_vel;
motor_get_velocity(&motor, &actual_vel);

// 失能
motor_disable(&motor);

// 反初始化
motor_deinit(&motor);
```

### 传感器读取示例

```c
#include "drivers/sensor_manager.h"

SensorManager_t sensor_mgr;

// 初始化
sensor_mgr_init(&sensor_mgr, "/dev/ttyUSB1", 115200);
sensor_mgr_start(&sensor_mgr);

// 读取编码器
SensorData_t encoder_data;
sensor_mgr_get_data(&sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data);

// 读取压力
SensorData_t pressure_data;
sensor_mgr_get_data(&sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure_data);
```

## 线程架构

| 线程 | 频率 | 功能 |
|------|------|------|
| data_collection_thread | 100Hz | 采集传感器数据到共享缓冲区 |
| data_output_thread | 20Hz | 输出数据到共享内存 |
| command_handler_thread | 100Hz | 处理上位机命令 |
| sensor_manager_thread | 50-100Hz | RS485传感器轮询 |

## 100Hz实时控制

系统通过NiMotion SDK实现100Hz PDO通信：
- PDO发送周期: 10ms (100Hz)
- Sync信号周期: 10ms (100Hz)
- 速度/位置命令通过PDO实时发送
- 实际位置/速度通过PDO实时读取
