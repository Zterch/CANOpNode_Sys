# CANOpNode_Sys - 多设备控制系统

## 系统概述

CANOpNode_Sys 是一个工业级多设备控制系统，集成了以下设备：
- **Nimotion CANopen伺服电机**（通过SocketCAN控制）
- **电源板**（UART/TTL串口控制电流输出）
- **编码器**（RS485接口）
- **压力计**（RS485接口）

## 系统架构

```
CANOpNode_Sys/
├── config/                 # 配置层
│   └── system_config.h     # 系统配置参数
├── drivers/                # 驱动层
│   ├── motor_driver.h      # 电机驱动接口
│   ├── power_driver.h      # 电源板驱动接口
│   ├── encoder_driver.h    # 编码器驱动接口
│   └── pressure_driver.h   # 压力计驱动接口
├── algorithms/             # 算法层
│   ├── sine_wave.h         # 正弦波生成算法
│   └── sine_wave.c
├── utils/                  # 工具层
│   ├── logger.h/.c         # 日志系统
│   └── thread_manager.h    # 线程管理器
├── main.c                  # 主程序
├── Makefile                # 构建系统
└── README.md               # 说明文档
```

### 架构特点

1. **分层设计**：清晰的层次结构，便于维护和扩展
2. **多线程架构**：控制、数据采集、日志分离
3. **线程安全**：所有共享资源使用互斥锁保护
4. **实时性**：支持Linux实时调度策略
5. **可配置**：集中式配置管理

## 编译运行

### 编译
```bash
cd /home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys
make
```

### 调试模式编译
```bash
make debug
```

### 运行
```bash
sudo ./bin/CANOpNode_Sys
```

或
```bash
make run
```

### 清理
```bash
make clean
```

## 配置说明

编辑 `config/system_config.h` 修改系统参数：

### 电机配置
```c
#define MOTOR_CAN_INTERFACE     "can0"      // CAN接口
#define MOTOR_NODE_ID           1           // 节点ID
#define MOTOR_CSV_AMPLITUDE     5000        // 正弦波幅度
#define MOTOR_CSV_FREQUENCY     0.5         // 正弦波频率
```

### 串口设备配置
```c
#define POWER_UART_DEVICE       "/dev/ttyUSB0"  // 电源板
#define ENCODER_UART_DEVICE     "/dev/ttyUSB1"  // 编码器
#define PRESSURE_UART_DEVICE    "/dev/ttyUSB1"  // 压力计
```

## 线程架构

| 线程 | 优先级 | 周期 | 功能 |
|------|--------|------|------|
| Control | 高(80) | 10ms | 电机控制、正弦波生成 |
| DataAcq | 中(50) | 10ms | 数据采集 |
| Logger | 低(20) | 100ms | 日志输出、状态监控 |

## 当前状态

### 已实现
- ✅ 系统架构搭建
- ✅ 日志系统
- ✅ 线程管理器
- ✅ 正弦波算法
- ✅ 电机驱动接口
- ✅ 主程序框架
- ✅ 多线程控制

### 待实现
- ⏳ 电机驱动实现（CANopen通信）
- ⏳ 电源板驱动实现
- ⏳ 编码器驱动实现
- ⏳ 压力计驱动实现
- ⏳ 数据处理算法
- ⏳ 数据记录功能

## 扩展指南

### 添加新设备驱动

1. 在 `drivers/` 目录创建 `xxx_driver.h` 和 `xxx_driver.c`
2. 实现标准接口：init/deinit/read/write
3. 在 `main.c` 中添加设备实例和初始化代码

### 添加新算法

1. 在 `algorithms/` 目录创建算法文件
2. 实现算法函数
3. 在控制线程中调用算法

## 许可证

MIT License

## 作者

System Architect
