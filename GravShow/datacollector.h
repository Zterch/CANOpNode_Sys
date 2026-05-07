#ifndef DATACOLLECTOR_H
#define DATACOLLECTOR_H

#include <QObject>
#include <QTimer>
#include <QTcpSocket>
#include "datamodel.h"

/**
 * @brief 数据采集器 - 从算法系统获取实时数据
 * 
 * 支持两种数据采集方式：
 * 1. 文件监控模式 - 监控算法输出的数据文件
 * 2. 网络模式 - 通过TCP/UDP接收数据（预留接口）
 */
class DataCollector : public QObject
{
    Q_OBJECT

public:
    explicit DataCollector(DataModel *model, QObject *parent = nullptr);
    ~DataCollector();

    // 开始采集
    void start(int intervalMs = 100);  // 默认100ms采集一次
    
    // 停止采集
    void stop();
    
    // 是否正在采集
    bool isRunning() const { return m_timer->isActive(); }
    
    // 设置数据源文件路径
    void setDataFilePath(const QString &path) { m_dataFilePath = path; }
    QString dataFilePath() const { return m_dataFilePath; }
    
    // 设置采集间隔
    void setInterval(int ms);
    int interval() const { return m_timer->interval(); }

signals:
    // 采集状态改变
    void started();
    void stopped();
    
    // 错误信号
    void error(const QString &message);
    
    // 连接状态
    void connected();
    void disconnected();

private slots:
    // 定时采集数据
    void collectData();

private:
    // 从文件解析数据
    bool parseDataFromFile();
    
    // 解析单行数据
    bool parseDataLine(const QString &line, SensorData &data);

private:
    DataModel *m_model;
    QTimer *m_timer;
    QString m_dataFilePath;
    
    // 数据文件路径
    static const QString DEFAULT_DATA_FILE;
};

#endif // DATACOLLECTOR_H
