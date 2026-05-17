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
        // 文件不存在，不报错，继续等待
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
        if (lines.size() > 10) {
            lines.removeFirst();  // 只保留最后10行
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
    // 格式1: [DATA] P=0.000kg Pos=1.311m V_raw=0.000 V_filt=0.000 I_clutch=50.0mA V_motor=0
    // 格式2: [STATUS] State=3 Error=0 Encoder=12345 MotorPos=67890 MotorSpeed=100
    // 格式3: CSV格式
    
    if (line.contains("[DATA]")) {
        // 解析 [DATA] 格式
        QRegularExpression re("P=([\\d.-]+)kg\\s+Pos=([\\d.-]+)m\\s+"
                          "V_raw=([\\d.-]+)\\s+V_filt=([\\d.-]+)\\s+"
                          "I_clutch=([\\d.-]+)");
        QRegularExpressionMatch match = re.match(line);
        
        if (match.hasMatch()) {
            data.pressure = match.captured(1).toDouble();
            data.ropeLength = match.captured(2).toDouble();
            // V_raw, V_filt 为原始速度和滤波速度
            // I_clutch 为离合器电流(mA)
            data.current = match.captured(5).toDouble() / 1000.0; // mA to A
            return true;
        }
    } else if (line.contains("[STATUS]")) {
        // 解析状态行
        QRegularExpression re("State=(\\d+)\\s+Error=(\\d+)\\s+"
                          "Encoder=(\\d+)\\s+MotorPos=([\\d.-]+)\\s+"
                          "MotorSpeed=([\\d.-]+)");
        QRegularExpressionMatch match = re.match(line);
        
        if (match.hasMatch()) {
            data.algorithmState = match.captured(1).toInt();
            data.algorithmError = match.captured(2).toInt();
            data.encoderValue = match.captured(3).toUInt();
            data.motorPosition = match.captured(4).toDouble();
            data.motorSpeed = match.captured(5).toDouble();
            return true;
        }
    } else if (line.contains("[MOTOR]")) {
        // 解析电机状态
        QRegularExpression re("Status=(\\w+)\\s+Speed=([\\d.-]+)");
        QRegularExpressionMatch match = re.match(line);
        
        if (match.hasMatch()) {
            data.motorStatusStr = match.captured(1);
            data.motorSpeed = match.captured(2).toDouble();
            
            // 解析状态码
            if (data.motorStatusStr == "Running") {
                data.motorStatus = 1;
            } else if (data.motorStatusStr == "Stopped") {
                data.motorStatus = 0;
            } else if (data.motorStatusStr == "Error") {
                data.motorStatus = -1;
            }
            return true;
        }
    } else if (line.contains(",")) {
        // 解析CSV格式
        QStringList parts = line.split(",");
        if (parts.size() >= 5) {
            // 基本字段
            data.current = parts[1].toDouble();
            data.voltage = parts[2].toDouble();
            data.pressure = parts[3].toDouble();
            data.ropeLength = parts[4].toDouble();
            
            // 扩展字段
            if (parts.size() >= 6) {
                data.encoderValue = parts[5].toUInt();
            }
            if (parts.size() >= 7) {
                data.encoderAngle = parts[6].toDouble();
            }
            if (parts.size() >= 8) {
                data.motorSpeed = parts[7].toDouble();
            }
            if (parts.size() >= 9) {
                data.motorPosition = parts[8].toDouble();
            }
            if (parts.size() >= 10) {
                data.motorStatus = parts[9].toInt();
            }
            if (parts.size() >= 11) {
                data.motorStatusStr = parts[10];
            }
            if (parts.size() >= 12) {
                data.algorithmState = parts[11].toInt();
            }
            if (parts.size() >= 13) {
                data.algorithmError = parts[12].toInt();
            }
            if (parts.size() >= 14) {
                data.emergencyStop = (parts[13].toInt() != 0);
            }
            return true;
        }
    }
    
    return false;
}
