#ifndef DATARECORDER_H
#define DATARECORDER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include "datamodel.h"

/**
 * @brief 数据记录器 - 将采集的数据保存到CSV文件
 */
class DataRecorder : public QObject
{
    Q_OBJECT

public:
    explicit DataRecorder(DataModel *model, QObject *parent = nullptr);
    ~DataRecorder();

    // 开始记录
    bool startRecording(const QString &filePath = QString());
    
    // 停止记录
    void stopRecording();
    
    // 是否正在记录
    bool isRecording() const { return m_isRecording; }
    
    // 获取当前记录文件路径
    QString currentFilePath() const { return m_currentFilePath; }
    
    // 获取记录的数据点数
    int recordedCount() const { return m_recordedCount; }
    
    // 设置自动保存目录
    void setLogDirectory(const QString &dir) { m_logDirectory = dir; }
    QString logDirectory() const { return m_logDirectory; }

signals:
    // 记录状态改变
    void recordingStarted(const QString &filePath);
    void recordingStopped();
    
    // 记录进度
    void dataRecorded(int count);
    
    // 错误信号
    void error(const QString &message);

private slots:
    // 数据更新时记录
    void onDataUpdated(const SensorData &data);

private:
    // 生成默认文件名
    QString generateDefaultFileName() const;
    
    // 写入CSV头部
    void writeCsvHeader();

private:
    DataModel *m_model;
    QFile *m_file;
    QTextStream *m_stream;
    
    bool m_isRecording;
    QString m_currentFilePath;
    QString m_logDirectory;
    int m_recordedCount;
};

#endif // DATARECORDER_H
