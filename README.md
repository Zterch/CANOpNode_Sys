# CANOpNode_Sys - 多设备控制系统

## 系统概述

CANOpNode_Sys 是一个工业级多设备控制系统，集成了以下设备：
- **Nimotion CANopen伺服电机**（通过SocketCAN控制）
- **电源板**（UART/TTL串口控制电流输出）
- **编码器**（RS485接口，Modbus RTU协议）
- **压力计**（RS485接口）

## 系统架构

```
CANOpNode_Sys/
├── config/                 # 配置层
│   └── system_config.h     # 系统配置参数
├── drivers/                # 驱动层
│   ├── motor_driver.h/.c   # 电机驱动（CANopen协议）✅
│   ├── power_driver.h/.c   # 电源板驱动（待实现）
│   ├── encoder_driver.h/.c # 编码器驱动（Modbus RTU）✅
│   └── pressure_driver.h/.c# 压力计驱动（待实现）
├── algorithms/             # 算法层
│   ├── sine_wave.h/.c      # 正弦波生成算法 ✅
├── utils/                  # 工具层
│   ├── logger.h/.c         # 日志系统 ✅
│   └── thread_manager.h/.c # 线程管理器 ✅
├── main.c                  # 主程序 ✅
├── Makefile                # 构建系统 ✅
└── README.md               # 说明文档
```

### 架构特点

1. **分层设计**：清晰的层次结构，便于维护和扩展
2. **多线程架构**：控制、数据采集、日志分离
3. **线程安全**：所有共享资源使用互斥锁保护
4. **实时性**：支持Linux实时调度策略
5. **可配置**：集中式配置管理

---

## 编译指南

### 环境要求

- Linux操作系统（推荐Ubuntu 18.04+）
- GCC编译器
- Make工具
- SocketCAN支持（用于CAN通信）

### 编译步骤

#### 1. 完整编译（推荐）

```bash
cd /home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys
make clean
make
```

#### 2. 仅重新编译修改的文件

如果只修改了部分源文件，直接运行：

```bash
make
```

Make会自动检测修改的文件并只编译变化的部分。

#### 3. 调试模式编译

```bash
make debug
```

调试模式会添加 `-g` 选项，便于使用GDB调试。

#### 4. 发布模式编译

```bash
make release
```

### 编译输出

编译成功后，可执行文件位于：
```
bin/CANOpNode_Sys
```

---

## 运行指南

### 运行前准备

#### 1. 配置硬件设备

编辑 `config/system_config.h` 确认设备配置：

```c
/* 编码器配置 */
#define ENCODER_UART_DEVICE     "/dev/ttyUSB1"  /* RS485设备 */
#define ENCODER_UART_BAUDRATE   9600
#define ENCODER_SLAVE_ADDR      1               /* Modbus地址 */
#define ENCODER_RESOLUTION      4096            /* 编码器分辨率 */

/* 电机配置 */
#define MOTOR_CAN_INTERFACE     "can0"
#define MOTOR_NODE_ID           1
```

#### 2. 设置设备权限

```bash
# 查看设备
ls -l /dev/ttyUSB*

# 添加用户到dialout组（需要重新登录生效）
sudo usermod -a -G dialout $USER

# 临时设置权限（立即生效）
sudo chmod 666 /dev/ttyUSB1
```

#### 3. 启动CAN接口（如使用电机）

```bash
# 设置CAN波特率为1Mbps
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 验证
ip link show can0
```

### 运行模式

#### 模式1：纯编码器测试（电机不上电）

```bash
sudo ./bin/CANOpNode_Sys --no-motor
```

或简写：
```bash
sudo ./bin/CANOpNode_Sys -n
```

**输出示例：**
```
[INFO]  [ENCODER] Pos=  322 Angle= 28.30° | Reads: 50, Errors: 0, Success: 100.0%
[INFO]  [ENCODER] Pos=  450 Angle= 39.55° | Reads: 50, Errors: 0, Success: 100.0%
```

#### 模式2：完整系统（电机+编码器）

```bash
sudo ./bin/CANOpNode_Sys
```

**注意：** 确保电机已上电并连接好CAN总线。

#### 模式3：使用make运行

```bash
make run
```

这会自动使用sudo运行程序。

### 停止程序

- 按 `Ctrl+C` 正常停止
- 程序会在10秒后自动停止（由 `DEMO_RUN_TIME_S` 配置）

---

## 修改代码后的编译流程

### 场景1：修改配置文件

```bash
# 编辑配置文件
vim config/system_config.h

# 重新编译（修改头文件会触发相关文件重新编译）
make
```

### 场景2：修改驱动代码

```bash
# 例如修改编码器驱动
vim drivers/encoder_driver.c

# 重新编译
make

# 如果修改了头文件，建议清理后重新编译
make clean && make
```

### 场景3：修改主程序

```bash
# 修改主程序
vim main.c

# 重新编译
make
```

### 场景4：添加新文件

```bash
# 1. 创建新文件
touch drivers/new_device.c

# 2. 修改Makefile，添加新文件到SOURCES
vim Makefile

# 3. 重新编译
make clean && make
```

---

## 配置说明

### 编码器参数配置

编辑 `config/system_config.h`：

```c
/* 编码器串口设备 */
#define ENCODER_UART_DEVICE     "/dev/ttyUSB1"

/* 波特率 */
#define ENCODER_UART_BAUDRATE   9600

/* Modbus从机地址 */
#define ENCODER_SLAVE_ADDR      1

/* 编码器分辨率（4096 = 12位） */
#define ENCODER_RESOLUTION      4096

/* 读取频率 */
#define ENCODER_READ_PERIOD_MS  10      /* 100Hz */
#define ENCODER_PRINT_PERIOD_MS 500     /* 2Hz */

/* Modbus寄存器地址 */
#define ENCODER_MODBUS_REG_ADDR 0x0000  /* 西门子PLC地址40001 */
```

### 电机参数配置

```c
/* CAN接口 */
#define MOTOR_CAN_INTERFACE     "can0"

/* CAN节点ID */
#define MOTOR_NODE_ID           1

/* CSV模式参数 */
#define MOTOR_CSV_AMPLITUDE     5000    /* 正弦波幅度 */
#define MOTOR_CSV_FREQUENCY     0.5     /* 正弦波频率(Hz) */
#define MOTOR_CSV_PERIOD_MS     10      /* 控制周期(ms) */
```

### 系统运行参数

```c
/* 运行时间（秒） */
#define DEMO_RUN_TIME_S         10

/* 日志级别 */
#define SYSTEM_LOG_LEVEL        3       /* 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG */
```

---

## 线程架构

| 线程 | 优先级 | 周期 | 功能 | 条件 |
|------|--------|------|------|------|
| EncoderData | 中(50) | 10ms | 100Hz读取编码器 | 始终运行 |
| EncoderPrint | 低(20) | 500ms | 2Hz打印编码器数据 | 始终运行 |
| Control | 高(80) | 10ms | 电机控制 | 电机模式启用 |

---

## 故障排查

### 问题1：编码器读取失败

**现象：**
```
[WARN]  Modbus response timeout or incomplete
```

**排查步骤：**
1. 检查设备连接：`ls -l /dev/ttyUSB1`
2. 检查设备权限：`sudo chmod 666 /dev/ttyUSB1`
3. 验证Modbus地址：`#define ENCODER_SLAVE_ADDR 1`
4. 检查波特率设置是否匹配

### 问题2：CAN接口错误

**现象：**
```
[ERROR] Failed to initialize CAN socket
```

**解决：**
```bash
# 启动CAN接口
sudo ip link set can0 up

# 或重新配置
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 问题3：权限不足

**现象：**
```
Failed to open /dev/ttyUSB1: Permission denied
```

**解决：**
```bash
# 临时方案
sudo chmod 666 /dev/ttyUSB1

# 永久方案（需重新登录）
sudo usermod -a -G dialout $USER
```

---

## 扩展指南

### 添加新设备驱动

1. **创建驱动文件**
   ```bash
   touch drivers/new_driver.h
   touch drivers/new_driver.c
   ```

2. **实现标准接口**
   ```c
   ErrorCode_t new_driver_init(NewDriver_t *dev, ...);
   void new_driver_deinit(NewDriver_t *dev);
   ErrorCode_t new_driver_read(NewDriver_t *dev, ...);
   ```

3. **修改Makefile**
   ```makefile
   SOURCES += drivers/new_driver.c
   ```

4. **在main.c中添加初始化**
   ```c
   new_driver_init(&g_new_dev, ...);
   ```

5. **重新编译**
   ```bash
   make clean && make
   ```

---

## 当前状态

### ✅ 已实现
- 系统架构搭建
- 日志系统（彩色输出、文件记录）
- 线程管理器（优先级、统计）
- 正弦波算法
- **电机驱动（完整CANopen实现）**
- **编码器驱动（Modbus RTU，100Hz读取，2Hz打印）**
- 多线程控制框架
- 构建系统

### ⏳ 待实现
- 电源板驱动（UART协议待补充）
- 压力计驱动（RS485协议待补充）
- 数据处理算法
- 数据记录功能

---

## 许可证

MIT License

## 作者

System Architect
