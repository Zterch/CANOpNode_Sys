#ifndef SHMDATACOLLECTOR_H
#define SHMDATACOLLECTOR_H

#include <QObject>
#include <QTimer>
#include <QDateTime>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <climits>  // for INT32_MAX, INT32_MIN

// 与下位机共享内存头文件保持一致的结构
#define SHM_NAME "/gravshow_shm"
#define SHM_SIZE 4096
#define SHM_MAGIC 0x47524156  // "GRAV"
#define DATA_VERSION 2

// 实时数据结构 - 与下位机完全一致
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint64_t timestamp_us;
    uint32_t sequence;
    
    float pressure_kg;
    float rope_length_m;
    uint32_t encoder_value;
    float encoder_angle_deg;
    
    float current_a;
    float voltage_v;
    
    float motor_speed_rpm;
    float motor_position;
    int32_t motor_status;
    
    int32_t algorithm_state;
    int32_t algorithm_error;
    bool emergency_stop;
    
    float clutch_current_ma;
    float motor_cmd_rpm;
    
    float reserved[4];
} SharedData_t;

// 控制命令结构
typedef struct {
    uint32_t magic;
    uint32_t command_id;
    
    int32_t cmd_type;
    float cmd_value;
    float cmd_accel;
    
    bool algorithm_start;
    bool algorithm_stop;
} SharedCommand_t;

// 数据偏移
#define DATA_OFFSET     0
#define COMMAND_OFFSET  1024

class DataModel;
class SensorData;

/**
 * @brief 共享内存数据收集器 - 20Hz实时数据采集
 * 
 * 基于POSIX共享内存实现零拷贝高速通信
 * - 数据频率：20Hz（50ms周期）
 * - 延迟：<1ms
 */
class ShmDataCollector : public QObject
{
    Q_OBJECT

public:
    explicit ShmDataCollector(DataModel *model, QObject *parent = nullptr);
    ~ShmDataCollector();

    // 开始采集
    void start(int intervalMs = 50);  // 默认50ms = 20Hz
    
    // 停止采集
    void stop();
    
    // 是否正在采集
    bool isRunning() const { return m_timer->isActive(); }
    
    // 设置采集间隔
    void setInterval(int ms);
    int interval() const { return m_timer->interval(); }
    
    // 共享内存状态
    bool isConnected() const { return m_shmConnected; }
    
    // 发送电机命令
    bool sendMotorCommand(int cmdType, double value, double accel = 0);
    bool sendAlgorithmStart();
    bool sendAlgorithmStop();
    
    // 发送电源板控制命令
    bool sendPowerCommand(int cmdType, double value);

signals:
    // 采集状态改变
    void started();
    void stopped();
    
    // 连接状态
    void connected();
    void disconnected();
    
    // 错误信号
    void error(const QString &message);
    
    // 数据接收统计
    void dataReceived(uint32_t sequence, uint64_t timestamp_us);

private slots:
    // 定时采集数据
    void collectData();

private:
    // 初始化共享内存
    bool initSharedMemory();
    
    // 关闭共享内存
    void closeSharedMemory();
    
    // 从共享内存读取数据
    bool readSharedData(SharedData_t *data);
    
    // 写入命令到共享内存
    bool writeCommand(SharedCommand_t *cmd);
    
    // 转换数据结构
    void convertToSensorData(const SharedData_t &sharedData, SensorData &sensorData);

private:
    DataModel *m_model;
    QTimer *m_timer;
    
    // 共享内存
    int m_shmFd;
    void *m_shmPtr;
    SharedData_t *m_dataPtr;
    SharedCommand_t *m_commandPtr;
    bool m_shmConnected;
    
    // 统计
    uint32_t m_lastSequence;
    int m_packetLossCount;
    int m_syncCount;  // 同步计数器
    uint32_t m_commandId;
};

#endif // SHMDATACOLLECTOR_H
