#ifndef DATAMODEL_H
#define DATAMODEL_H

#include <QObject>
#include <QDateTime>
#include <QVector>

/**
 * @brief 传感器数据结构
 */
struct SensorData {
    QDateTime timestamp;
    double current;         // 电流 (A)
    double voltage;         // 电压 (V)
    double pressure;        // 压力 (kg)
    double ropeLength;      // 绳长 (m)
    double motorSpeed;      // 电机速度 (rpm)
    
    SensorData() : current(0), voltage(0), pressure(0), ropeLength(0), motorSpeed(0) {}
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

signals:
    // 数据更新信号
    void dataUpdated(const SensorData &data);
    
    // 历史数据重置信号
    void historyCleared();

private:
    SensorData m_latestData;
    QVector<SensorData> m_historyData;
    int m_maxCapacity = 10000;  // 最大存储10000个数据点
};

#endif // DATAMODEL_H
