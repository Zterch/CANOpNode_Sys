# 重力卸载控制系统算法文档

## 系统概述

本系统实现了一个完整的重力卸载控制系统，通过协调电机和磁粉离合器来动态平衡悬挂重物的重力。

### 系统组成

```
重物 → 绳子 → 滑轮2(R2=50mm) → 编码器+压力传感器
                          ↓
                    斜向绳子
                          ↓
            滑轮1(R1=100mm) → 电机 + 磁粉离合器 → 卷曲弹簧片
```

### 控制目标

- 弹簧拉力抵消大部分重力
- 磁粉离合器通过电流控制输出扭矩来平衡力的波动
- 电机通过PID控制跟踪重物运动

---

## 算法架构

### 核心算法流程

```
┌─────────────────────────────────────────────────────────────┐
│                     控制周期 (10ms = 100Hz)                  │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│ 1. 读取传感器数据                                            │
│    ├── 压力传感器：获取当前张力 (kg)                         │
│    └── 编码器：获取重物位置 (m)                              │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. 传感器数据处理                                            │
│    ├── 压力低通滤波                                          │
│    ├── 压力变化率计算                                        │
│    ├── 位置数据                                              │
│    └── 速度计算（移动平均滤波，0.05s窗口）                   │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. 控制输出计算                                              │
│    ├── 离合器电流：I = F × R2 × K_clutch                    │
│    │   (F=压力×9.81, R2=0.05m, K_clutch=176 mA/Nm)          │
│    └── 电机速度：ω = (V/R1) × (1+C) + PID                  │
│        (V=重物速度, R1=0.1m, C=0.1补偿系数)                 │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. 安全检查                                                  │
│    ├── 压力范围检查                                          │
│    ├── 位置范围检查                                          │
│    ├── 速度限制检查                                          │
│    ├── 离合器电流限制                                        │
│    └── 错误计数（连续10次错误触发紧急停止）                  │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. 输出到执行器                                              │
│    ├── 设置离合器电流                                        │
│    └── 设置电机速度                                          │
└─────────────────────────────────────────────────────────────┘
```

---

## 关键参数配置

所有参数集中在 `algorithms/algorithm_config.h` 中配置：

### 物理系统参数

```c
#define PULLEY_R1_MOTOR_RADIUS_M        0.100f      /* 电机侧滑轮半径 100mm */
#define PULLEY_R2_ENCODER_RADIUS_M      0.050f      /* 编码器侧滑轮半径 50mm */
#define CLUTCH_RATED_CURRENT_MA         880         /* 离合器额定电流 880mA */
#define CLUTCH_RATED_TORQUE_NM          5.0f        /* 离合器额定转矩 5Nm */
```

### 控制参数

```c
#define ALGO_CONTROL_PERIOD_MS          10          /* 控制周期 10ms */
#define SPEED_FILTER_WINDOW_SIZE        5           /* 速度滤波窗口 */
#define MOTOR_SPEED_COMPENSATION_C      0.1f        /* 电机速度补偿 10% */
```

### PID参数

```c
#define PID_KP                          1.0f        /* 比例系数 */
#define PID_KI                          0.1f        /* 积分系数 */
#define PID_KD                          0.01f       /* 微分系数 */
```

### 安全限制

```c
#define SAFETY_PRESSURE_MIN_KG          -0.5f       /* 最小压力 */
#define SAFETY_PRESSURE_MAX_KG          10.0f       /* 最大压力 */
#define SAFETY_MOTOR_SPEED_MAX          8000        /* 电机最大速度 */
#define SAFETY_CLUTCH_CURRENT_MAX_MA    900         /* 离合器最大电流 */
```

---

## 算法模块说明

### 1. 信号滤波模块 (signal_filter.h/c)

**移动平均滤波器**：用于速度信号滤波
```c
MovingAverageFilter_t velocity_filter;
ma_filter_init(&velocity_filter);
float filtered_vel = ma_filter_update(&velocity_filter, raw_velocity);
```

**低通滤波器**：用于压力信号滤波
```c
LowPassFilter_t pressure_filter;
lpf_init(&pressure_filter, 0.8f);  /* alpha = 0.8 */
float filtered_pressure = lpf_update(&pressure_filter, raw_pressure);
```

**微分器**：计算变化率
```c
Differentiator_t diff;
diff_init(&diff, 0.5f);
float rate = diff_update(&diff, value, timestamp_ms);
```

### 2. PID控制器 (pid_controller.h/c)

```c
PID_Controller_t pid;
pid_init(&pid, 1.0f, 0.1f, 0.01f, -10000, 10000, 0.01f);

float output = pid_update(&pid, setpoint, measurement);
```

### 3. 安全监控 (safety_monitor.h/c)

实时监控所有关键参数：
- 压力范围和变化率
- 位置和速度限制
- 电机速度限制
- 离合器电流限制

连续10次错误触发紧急停止。

### 4. 重力卸载算法 (gravity_unload.h/c)

核心控制算法实现：
- 初始化配置
- 传感器数据处理
- 控制输出计算
- 安全监控
- 线程管理

---

## 启动流程

```
┌─────────────────┐
│  1. 初始化硬件   │
│  - 传感器管理器  │
│  - 电源板       │
│  - 电机         │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 2. 系统预检测    │
│ - 传感器数据    │
│ - 电机状态      │
│ - 电源板通信    │
│ - 安全检查      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 3. 用户确认      │
│ 显示安全提示    │
│ 等待用户输入yes │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 4. 启动算法      │
│ 初始化算法参数  │
│ 启动控制线程    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 5. 实时监控      │
│ 100Hz控制循环   │
│ 1Hz状态打印     │
└─────────────────┘
```

---

## 使用说明

### 编译运行

```bash
cd CANOpNode_Sys
make clean && make
sudo ./bin/CANOpNode_Sys
```

### 命令行选项

```bash
sudo ./bin/CANOpNode_Sys        # 正常启动（电机使能）
sudo ./bin/CANOpNode_Sys -n     # 禁用电机（仅传感器测试）
```

### 启动确认

程序启动后会显示安全提示，需要输入 `yes` 确认：

```
╔══════════════════════════════════════════════════════════════╗
║         重力卸载控制系统 - 启动确认                          ║
╠══════════════════════════════════════════════════════════════╣
║  ⚠️  系统即将启动重力卸载算法                                ║
║                                                              ║
║  请确认：                                                    ║
║  1. 所有设备已正确连接                                       ║
║  2. 重物已正确悬挂                                           ║
║  3. 人员已远离运动区域                                       ║
║  4. 紧急情况可以按 Ctrl+C 停止                               ║
╚══════════════════════════════════════════════════════════════╝

请输入 'yes' 启动算法，或其他键取消: 
```

---

## 扩展与维护

### 修改物理参数

编辑 `algorithms/algorithm_config.h`：

```c
/* 修改滑轮半径 */
#define PULLEY_R1_MOTOR_RADIUS_M        0.120f      /* 改为120mm */

/* 修改离合器参数 */
#define CLUTCH_RATED_CURRENT_MA         1000        /* 改为1A */
```

### 调整PID参数

```c
/* 调整PID参数 */
#define PID_KP                          2.0f        /* 增大比例系数 */
#define PID_KI                          0.05f       /* 减小积分系数 */
```

### 修改安全限制

```c
/* 调整安全范围 */
#define SAFETY_PRESSURE_MAX_KG          15.0f       /* 允许更大压力 */
#define SAFETY_MOTOR_SPEED_MAX          10000       /* 允许更高转速 */
```

---

## 故障排查

### 1. 传感器数据异常

```
[CHECK FAIL] Pressure sensor data not available
```

**解决**：
- 检查串口设备 `/dev/ttyUSB*` 权限
- 检查传感器Modbus地址配置
- 检查物理连接

### 2. 电机通信失败

```
[MOTOR] ERROR: Failed to enable motor
```

**解决**：
- 检查CAN接口状态：`ip link show can0`
- 确认电机NodeID配置
- 检查电机电源

### 3. 安全限制触发

```
[EMERGENCY] Algorithm stopped due to safety violation!
```

**解决**：
- 检查 `safety_monitor.c` 中的错误信息
- 调整 `algorithm_config.h` 中的安全限制
- 检查传感器校准

---

## 文件结构

```
algorithms/
├── algorithm_config.h     # 算法参数配置
├── signal_filter.h/c      # 信号滤波模块
├── pid_controller.h/c     # PID控制器
├── safety_monitor.h/c     # 安全监控
├── gravity_unload.h/c     # 重力卸载核心算法
└── system_check.h/c       # 系统预检测

main.c                     # 主程序集成
```
