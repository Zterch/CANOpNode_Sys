#include "datarecorder.h"
#include <QDir>
#include <QDebug>

DataRecorder::DataRecorder(DataModel *model, QObject *parent)
    : QObject(parent)
    , m_model(model)
    , m_file(nullptr)
    , m_stream(nullptr)
    , m_isRecording(false)
    , m_recordedCount(0)
{
    // 默认日志目录
    m_logDirectory = "/home/zterch/VS_Project/Nimo_COp_Prj/GravShow/logdata";
    
    // 连接数据更新信号
    connect(m_model, &DataModel::dataUpdated, this, &DataRecorder::onDataUpdated);
}

DataRecorder::~DataRecorder()
{
    stopRecording();
}

bool DataRecorder::startRecording(const QString &filePath)
{
    if (m_isRecording) {
        stopRecording();
    }
    
    // 确定文件路径
    QString targetPath = filePath;
    if (targetPath.isEmpty()) {
        targetPath = generateDefaultFileName();
    }
    
    // 确保目录存在
    QFileInfo fileInfo(targetPath);
    QDir dir = fileInfo.dir();
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            emit error(QString("无法创建目录: %1").arg(dir.absolutePath()));
            return false;
        }
    }
    
    // 创建文件
    m_file = new QFile(targetPath, this);
    if (!m_file->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        emit error(QString("无法创建文件: %1").arg(targetPath));
        delete m_file;
        m_file = nullptr;
        return false;
    }
    
    m_stream = new QTextStream(m_file);
    m_stream->setCodec("UTF-8");
    
    // 写入CSV头部
    writeCsvHeader();
    
    m_isRecording = true;
    m_currentFilePath = targetPath;
    m_recordedCount = 0;
    
    emit recordingStarted(targetPath);
    qDebug() << "Recording started:" << targetPath;
    
    return true;
}

void DataRecorder::stopRecording()
{
    if (!m_isRecording) {
        return;
    }
    
    m_isRecording = false;
    
    if (m_stream) {
        m_stream->flush();
        delete m_stream;
        m_stream = nullptr;
    }
    
    if (m_file) {
        m_file->close();
        delete m_file;
        m_file = nullptr;
    }
    
    emit recordingStopped();
    qDebug() << "Recording stopped, total records:" << m_recordedCount;
}

void DataRecorder::onDataUpdated(const SensorData &data)
{
    if (!m_isRecording || !m_stream) {
        return;
    }
    
    // 写入CSV数据行
    *m_stream << data.timestamp.toString("yyyy-MM-dd hh:mm:ss.zzz") << ","
              << data.current << ","
              << data.voltage << ","
              << data.pressure << ","
              << data.ropeLength << ","
              << data.motorSpeed << "\n";
    
    m_stream->flush();  // 立即写入文件
    
    m_recordedCount++;
    emit dataRecorded(m_recordedCount);
}

QString DataRecorder::generateDefaultFileName() const
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    return QString("%1/gravity_data_%2.csv").arg(m_logDirectory).arg(timestamp);
}

void DataRecorder::writeCsvHeader()
{
    if (m_stream) {
        *m_stream << "Timestamp,Current(A),Voltage(V),Pressure(kg),RopeLength(m),MotorSpeed(rpm)\n";
        m_stream->flush();
    }
}
