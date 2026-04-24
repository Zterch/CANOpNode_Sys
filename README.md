# CANOpNode_Sys - 多设备控制系统

## 系统概述

CANOpNode_Sys 是一个工业级多设备控制系统，集成了以下设备：
- **Nimotion CANopen伺服电机**（通过SocketCAN控制）
- **电源板**（UART/TTL串口控制电流输出 0-4A）
- **编码器**（RS485接口，Modbus RTU协议，地址1）
- **压力计**（RS485接口，Modbus RTU协议，地址2，3位小数精度）

## 系统架构

```
CANOpNode_Sys/
├── config/                 # 配置层
│   └── system_config.h     # 系统配置参数
├── drivers/                # 驱动层
│   ├── motor_driver.h/.c   # 电机驱动（CANopen协议，自动配置CAN接口）✅
│   ├── power_driver.h/.c   # 电源板驱动（自定义协议）✅
│   ├── sensor_manager.h/.c # 统一传感器管理器（编码器+压力传感器）✅
│   ├── encoder_driver.h/.c # 编码器驱动（Modbus RTU）✅
│   ├── pressure_driver.h/.c# 压力计驱动（Modbus RTU）✅
│   └── rs485_bus.h/.c      # RS485总线管理器 ✅
├── algorithms/             # 算法层
│   ├── sine_wave.h/.c      # 正弦波生成算法 ✅
├── utils/                  # 工具层
│   ├── logger.h/.c         # 日志系统 ✅
│   └── thread_manager.h/.c # 线程管理器 ✅
├── share/                  # 数据文件
│   ├── axis.txt            # 轴数据
│   └── encoder_rope_data.txt # 编码器绳子长度数据
├── main.c                  # 主程序 ✅
├── Makefile                # 构建系统 ✅
└── README.md               # 说明文档
```

### 架构特点

1. **分层设计**：清晰的层次结构，便于维护和扩展
2. **统一传感器管理**：单一管理器轮询编码器和压力传感器，减少线程数量，避免RS485总线竞争
3. **线程安全**：双互斥锁设计（bus_mutex + data_mutex），确保数据安全和实时性
4. **实时性**：支持Linux实时调度策略
5. **可配置**：集中式配置管理
6. **自动CAN配置**：电机驱动自动配置CAN接口，无需手动设置

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

```bash
make
```

#### 3. 调试模式编译

```bash
make debug
```

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
/* 统一传感器管理器配置（RS485） */
#define ENCODER_UART_DEVICE     "/dev/ttyUSB2"  /* 编码器+压力传感器共用 */
#define ENCODER_UART_BAUDRATE   115200

/* 编码器配置（地址2） */
#define ENCODER_SLAVE_ADDR      2
#define ENCODER_RESOLUTION      4096

/* 压力传感器配置（地址1） */
#define PRESSURE_SLAVE_ADDR     1
#define PRESSURE_DECIMAL_PLACES 3               /* 3位小数 */

/* 电源板配置（UART/TTL） */
#define POWER_UART_DEVICE       "/dev/ttyUSB0"
#define POWER_UART_BAUDRATE     115200

/* 电机配置 */
#define MOTOR_CAN_INTERFACE     "can0"
#define MOTOR_NODE_ID           1
#define MOTOR_CAN_BITRATE       1000000         /* 1 Mbps，自动配置 */
```

#### 2. 设置设备权限

```bash
# 查看设备
ls -l /dev/ttyUSB*

# 添加用户到dialout组（需要重新登录生效）
sudo usermod -a -G dialout $USER

# 临时设置权限（立即生效）
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
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

#### 模式1：传感器测试模式（电机不上电）

```bash
sudo ./bin/CANOpNode_Sys --no-motor
```

或简写：
```bash
sudo ./bin/CANOpNode_Sys -n
```

**输出示例：**
```
[INFO] [ENCODER] Pos=  322 Angle= 28.30° | Reads: 50, Errors: 0, Success: 100.0%
[INFO] [PRESSURE] Pressure=  1.234 kg
[INFO] [POWER] Voltage=24.00V Current=0.44A | Reads: 50
...
[INFO] [POWER] Current set to 600 mA (0.60 A)  /* 5秒后自动设置 */
```

#### 模式2：完整系统（电机+所有传感器）

```bash
sudo ./bin/CANOpNode_Sys
```

#### 模式3：使用make运行

```bash
make run
```

### 停止程序

- 按 `Ctrl+C` 正常停止
- 程序会在10秒后自动停止（由 `DEMO_RUN_TIME_S` 配置）

---

## 修改代码后的编译流程

### 场景1：修改配置文件

```bash
vim config/system_config.h
make
```

### 场景2：修改驱动代码

```bash
vim drivers/pressure_driver.c
make

# 如果修改了头文件，建议清理后重新编译
make clean && make
```

### 场景3：修改主程序

```bash
vim main.c
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

```c
/* 编码器串口设备 - 与压力传感器共用 */
#define ENCODER_UART_DEVICE     "/dev/ttyUSB2"

/* 波特率 */
#define ENCODER_UART_BAUDRATE   115200

/* Modbus从机地址 */
#define ENCODER_SLAVE_ADDR      2

/* 编码器分辨率（4096 = 12位） */
#define ENCODER_RESOLUTION      4096

/* 绳子长度计算参数 */
#define ROPE_DRUM_DIAMETER      100.0f  /* 卷筒直径(mm) */
```

### 压力传感器参数配置

```c
/* 压力传感器串口设备 - 与编码器共用 */
#define PRESSURE_UART_DEVICE    "/dev/ttyUSB2"

/* 波特率 */
#define PRESSURE_UART_BAUDRATE  115200

/* Modbus从机地址 */
#define PRESSURE_SLAVE_ADDR     1       /* 设备地址1 */

/* 数据解析 */
#define PRESSURE_DECIMAL_PLACES 3       /* 3位小数，分辨率0.001kg */

/* 去皮功能 */
// 程序启动时自动执行去皮，保存当前值为零点
sensor_mgr_pressure_tare(&g_sensor_mgr);
```

### 统一传感器管理器配置

```c
/* 统一传感器管理器 - 编码器+压力传感器 */
#define SENSOR_MANAGER_DEVICE   "/dev/ttyUSB2"
#define SENSOR_MANAGER_BAUDRATE 115200

/* 编码器配置 */
#define ENCODER_SLAVE_ADDR      2
#define ENCODER_REG_ADDR        0x0000  /* 多圈值寄存器 */
#define ENCODER_REG_COUNT       2       /* 2个寄存器 = 4字节 */

/* 压力传感器配置 */
#define PRESSURE_SLAVE_ADDR     1
#define PRESSURE_REG_ADDR       0x0000  /* 压力值寄存器 */
#define PRESSURE_REG_COUNT      1       /* 1个寄存器 = 2字节 */
```

### 电源板参数配置

```c
/* 电源板串口设备 */
#define POWER_UART_DEVICE       "/dev/ttyUSB0"

/* 波特率 */
#define POWER_UART_BAUDRATE     115200

/* 电流范围 */
#define POWER_MAX_CURRENT       4000    /* 最大4A */
#define POWER_MIN_CURRENT       50      /* 最小0.05A */
#define POWER_DEFAULT_CURRENT   440     /* 默认0.44A */

/* 读取频率 */
#define POWER_READ_PERIOD_MS    20      /* 50Hz */
#define POWER_PRINT_PERIOD_MS   1000    /* 1Hz */
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
#define SYSTEM_LOG_LEVEL        2       /* 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG */
```

---

## 线程架构

系统使用精简的多线程架构：

| 线程名 | 类型 | 周期 | 功能 |
|--------|------|------|------|
| SensorManager | DATA | 20ms | 统一传感器管理器（50Hz轮询编码器+压力传感器）|
| Print | LOG | 1000ms | 1Hz打印所有传感器数据 |
| PowerTest | CONTROL | - | 5秒后设置电流0.6A |
| MotorControl | CONTROL | 10ms | 电机正弦波控制（可选）|

### 统一传感器管理器特点

- **单一RS485总线管理**：编码器和压力传感器共用串口，通过时间片轮询（各10ms）
- **双互斥锁设计**：
  - `bus_mutex`：保护串口访问
  - `data_mutex`：保护数据读取
- **50Hz更新频率**：20ms周期，避免总线竞争
- **支持去皮功能**：压力传感器自动去皮校准

---

## 故障排查

### 问题1：编码器读取失败

**现象：**
```
[WARN] Modbus response timeout or incomplete
```

**排查步骤：**
1. 检查设备连接：`ls -l /dev/ttyUSB1`
2. 检查设备权限：`sudo chmod 666 /dev/ttyUSB1`
3. 验证Modbus地址：编码器地址1，压力计地址2
4. 检查波特率设置是否匹配（默认115200）

### 问题2：压力计小数点设置失败

**现象：**
```
[WARN] Failed to set decimal places to 3
```

**排查步骤：**
1. 确认压力计设备地址已设置为2
2. 检查RS485总线连接
3. 尝试断电重启压力计

### 问题3：电源板通信失败

**现象：**
```
[ERROR] Failed to open /dev/ttyUSB0
```

**排查步骤：**
1. 检查UART/TTL连接
2. 确认设备节点：`ls -l /dev/ttyUSB*`
3. 检查波特率是否匹配（默认115200）

### 问题4：线程创建失败

**现象：**
```
[ERROR] Maximum thread count (16) reached
```

**解决：** 系统最多支持16个线程，请检查线程配置。

### 问题5：压力传感器数据异常

**现象：**
```
[PRESSURE] Pressure= 65.462 kg raw=65462
```

**原因：** 压力传感器数据为16位有符号整数，直接显示为无符号数导致数值错误。

**解决：** 已修复为正确处理有符号数据，并添加去皮功能。去皮后应显示正确的重量值。

### 问题6：CAN接口错误

**现象：**
```
[ERROR] Failed to initialize CAN socket
```

**解决：**
- 程序已自动配置CAN接口，无需手动设置
- 如需手动配置：
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

---

## GitHub 推送指南

### 首次推送（已配置）

```bash
cd /home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys

# 1. 查看修改状态
git status

# 2. 添加所有修改
git add -A

# 3. 提交修改
git commit -m "更新说明：添加电源板驱动和压力计小数点设置"

# 4. 推送到GitHub（使用token，将YOUR_TOKEN替换为实际token）
git remote set-url origin https://YOUR_TOKEN@github.com/Zterch/CANOpNode_Sys.git
git push origin main

# 5. （可选）清除token缓存
git remote set-url origin https://github.com/Zterch/CANOpNode_Sys.git
```

### 后续手动推送步骤

#### 方法1：使用Personal Access Token（推荐）

```bash
# 1. 进入项目目录
cd /home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys

# 2. 查看当前修改
git status

# 3. 添加修改的文件（全部添加）
git add -A

# 或添加指定文件
git add config/system_config.h
git add drivers/pressure_driver.c

# 4. 提交修改
git commit -m "修改说明"

# 5. 设置远程URL（带token）
git remote set-url origin https://YOUR_TOKEN@github.com/Zterch/CANOpNode_Sys.git

# 6. 推送到main分支
git push origin main

# 7. 清除token（安全起见）
git remote set-url origin https://github.com/Zterch/CANOpNode_Sys.git
```

#### 方法2：使用SSH密钥（更安全，长期有效）

```bash
# 1. 生成SSH密钥（如未生成）
ssh-keygen -t ed25519 -C "your_email@example.com"

# 2. 添加公钥到GitHub
# 复制 ~/.ssh/id_ed25519.pub 内容到 GitHub -> Settings -> SSH and GPG keys -> New SSH key

# 3. 修改远程URL为SSH格式
git remote set-url origin git@github.com:Zterch/CANOpNode_Sys.git

# 4. 后续推送只需
git add -A
git commit -m "修改说明"
git push origin main
```

#### 方法3：使用Git Credential Helper（缓存密码）

```bash
# 配置credential缓存（15分钟）
git config credential.helper cache

# 或永久存储（存储在~/.git-credentials，注意安全性）
git config credential.helper store

# 推送时会提示输入用户名和token，之后会被缓存
git push origin main
```

### 常用Git命令速查

```bash
# 查看状态
git status

# 查看修改内容
git diff

# 查看提交历史
git log --oneline -10

# 拉取远程更新（如有冲突需解决）
git pull origin main

# 强制推送（慎用，会覆盖远程）
git push origin main --force

# 查看远程URL
git remote -v
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
- **统一传感器管理器（编码器+压力传感器，单一管理器轮询）**
- 正弦波算法
- **电机驱动（完整CANopen实现，自动配置CAN接口）**
- **编码器驱动（Modbus RTU，多圈值读取，绳子长度计算）**
- **压力传感器驱动（Modbus RTU，有符号数据解析，去皮功能）**
- **电源板驱动（自定义协议，50Hz读取）**
- **RS485总线管理器（编码器+压力传感器共用）**
- 多线程控制框架
- 构建系统

### ⏳ 待实现
- 数据处理算法
- 数据记录功能
- 网络通信接口

---

## 许可证

MIT License

## 作者

System Architect

## 仓库地址

https://github.com/Zterch/CANOpNode_Sys.git
