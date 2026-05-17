#ifndef DATAMODEL_H
#define DATAMODEL_H

#include <QObject>
#include <QDateTime>
#include <QVector>

/**
 * @brief 传感器数据结构 - 扩展版本
 */
struct SensorData {
    QDateTime timestamp;
    
    // 基本传感器数据
    double current;         // 电流 (A)
    double voltage;         // 电压 (V)
    double pressure;        // 压力 (kg)
    double ropeLength;      // 绳长 (m)
    
    // 编码器数据
    uint32_t encoderValue;  // 编码器原始值
    double encoderAngle;    // 编码器角度 (度)
    
    // 电机数据
    double motorSpeed;      // 电机速度 (rpm)
    double motorPosition;   // 电机位置 (counts或圈数)
    int motorStatus;        // 电机状态码
    QString motorStatusStr; // 电机状态字符串
    
    // 算法状态
    int algorithmState;     // 算法状态
    int algorithmError;     // 算法错误码
    bool emergencyStop;     // 紧急停止标志
    
    SensorData() : 
        current(0), voltage(0), pressure(0), ropeLength(0),
        encoderValue(0), encoderAngle(0),
        motorSpeed(0), motorPosition(0), motorStatus(0),
        algorithmState(0), algorithmError(0), emergencyStop(false) {}
};

/**
 * @brief 电机控制命令结构
 */
struct MotorCommand {
    enum CommandType {
        CMD_NONE,
        CMD_VELOCITY,   // 速度模式
        CMD_POSITION    // 位置模式
    };
    
    CommandType type;
    double value;       // 速度值(rpm)或位置值(counts)
    double acceleration; // 加速度
    
    MotorCommand() : type(CMD_NONE), value(0), acceleration(0) {}
};

/**
 * @brief 数据模型类 - 管理实时数据和历史数据
 */
class DataModel : public QObject
{
    Q_OBJECT

public:
    explicit DataModel(QObject *parent = nullptr);
    ~DataModel();

    // 更新最新数据
    void updateData(const SensorData &data);
    
    // 获取最新数据
    SensorData latestData() const { return m_latestData; }
    
    // 获取历史数据
    QVector<SensorData> historyData() const { return m_historyData; }
    
    // 获取指定时间范围的数据
    QVector<SensorData> getDataInRange(const QDateTime &start, const QDateTime &end) const;
    
    // 清空历史数据
    void clearHistory();
    
    // 获取数据点数量
    int dataCount() const { return m_historyData.size(); }
    
    // 获取最大数据容量
    int maxCapacity() const { return m_maxCapacity; }
    void setMaxCapacity(int capacity) { m_maxCapacity = capacity; }
    
    // 电机控制命令
    void setMotorCommand(const MotorCommand &cmd) { m_motorCommand = cmd; }
    MotorCommand motorCommand() const { return m_motorCommand; }
    
    // 算法运行状态
    bool isAlgorithmRunning() const { return m_algorithmRunning; }
    void setAlgorithmRunning(bool running) { m_algorithmRunning = running; }

signals:
    // 数据更新信号
    void dataUpdated(const SensorData &data);
    
    // 历史数据重置信号
    void historyCleared();
    
    // 电机控制命令信号
    void motorControlRequested(const MotorCommand &cmd);

private:
    SensorData m_latestData;
    QVector<SensorData> m_historyData;
    int m_maxCapacity = 10000;  // 最大存储10000个数据点
    
    MotorCommand m_motorCommand;
    bool m_algorithmRunning = false;
};

#endif // DATAMODEL_H
