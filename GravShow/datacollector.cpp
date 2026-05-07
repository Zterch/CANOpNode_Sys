#include "datacollector.h"
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>
#include <QDebug>

const QString DataCollector::DEFAULT_DATA_FILE = "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/share/realtime_data.txt";

DataCollector::DataCollector(DataModel *model, QObject *parent)
    : QObject(parent)
    , m_model(model)
    , m_dataFilePath(DEFAULT_DATA_FILE)
{
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &DataCollector::collectData);
}

DataCollector::~DataCollector()
{
    stop();
}

void DataCollector::start(int intervalMs)
{
    if (!m_timer->isActive()) {
        m_timer->start(intervalMs);
        emit started();
        qDebug() << "DataCollector started, interval:" << intervalMs << "ms";
    }
}

void DataCollector::stop()
{
    if (m_timer->isActive()) {
        m_timer->stop();
        emit stopped();
        qDebug() << "DataCollector stopped";
    }
}

void DataCollector::setInterval(int ms)
{
    m_timer->setInterval(ms);
}

void DataCollector::collectData()
{
    if (!parseDataFromFile()) {
        // 解析失败，但不停止采集，继续尝试
    }
}

bool DataCollector::parseDataFromFile()
{
    QFile file(m_dataFilePath);
    if (!file.exists()) {
        // 文件不存在，创建示例数据文件
        return false;
    }
    
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        emit error(QString("无法打开数据文件: %1").arg(m_dataFilePath));
        return false;
    }
    
    QTextStream in(&file);
    
    // 读取文件最后几行（最新的数据）
    QStringList lines;
    while (!in.atEnd()) {
        lines.append(in.readLine());
        if (lines.size() > 5) {
            lines.removeFirst();  // 只保留最后5行
        }
    }
    file.close();
    
    // 解析最新数据行
    if (!lines.isEmpty()) {
        SensorData data;
        data.timestamp = QDateTime::currentDateTime();
        
        // 尝试解析每一行，使用最新的有效数据
        for (int i = lines.size() - 1; i >= 0; --i) {
            if (parseDataLine(lines[i], data)) {
                m_model->updateData(data);
                return true;
            }
        }
    }
    
    return false;
}

bool DataCollector::parseDataLine(const QString &line, SensorData &data)
{
    // 支持的格式1: [DATA] P=0.000kg Pos=1.311m V_raw=0.000 V_filt=0.000 I_clutch=50.0mA V_motor=0
    // 支持的格式2: CSV格式: timestamp,current,voltage,pressure,ropeLength,motorSpeed
    
    if (line.contains("[DATA]")) {
        // 解析 [DATA] 格式
        QRegularExpression re("P=([\\d.]+)kg\\s+Pos=([\\d.]+)m\\s+V_raw=([\\d.]+)\\s+V_filt=([\\d.]+)\\s+I_clutch=([\\d.]+)");
        QRegularExpressionMatch match = re.match(line);
        
        if (match.hasMatch()) {
            data.pressure = match.captured(1).toDouble();
            data.ropeLength = match.captured(2).toDouble();
            // V_raw, V_filt 为原始速度和滤波速度
            // I_clutch 为离合器电流(mA)
            return true;
        }
    } else if (line.contains(",")) {
        // 解析CSV格式
        QStringList parts = line.split(",");
        if (parts.size() >= 5) {
            data.current = parts[1].toDouble();
            data.voltage = parts[2].toDouble();
            data.pressure = parts[3].toDouble();
            data.ropeLength = parts[4].toDouble();
            if (parts.size() >= 6) {
                data.motorSpeed = parts[5].toDouble();
            }
            return true;
        }
    }
    
    return false;
}
