#include "datamodel.h"

DataModel::DataModel(QObject *parent) : QObject(parent)
{
    m_historyData.reserve(m_maxCapacity);
}

DataModel::~DataModel()
{
}

void DataModel::updateData(const SensorData &data)
{
    m_latestData = data;
    
    // 添加到历史数据
    m_historyData.append(data);
    
    // 限制历史数据容量
    if (m_historyData.size() > m_maxCapacity) {
        m_historyData.removeFirst();
    }
    
    emit dataUpdated(data);
}

QVector<SensorData> DataModel::getDataInRange(const QDateTime &start, const QDateTime &end) const
{
    QVector<SensorData> result;
    
    for (const auto &data : m_historyData) {
        if (data.timestamp >= start && data.timestamp <= end) {
            result.append(data);
        }
    }
    
    return result;
}

void DataModel::clearHistory()
{
    m_historyData.clear();
    emit historyCleared();
}
