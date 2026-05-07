#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "datamodel.h"
#include "datacollector.h"
#include "datarecorder.h"
#include "chartwidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/**
 * @brief 重力卸载系统数据监控主窗口
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // 开始/停止采集
    void onStartClicked();
    void onStopClicked();
    
    // 记录控制
    void onRecordStartClicked();
    void onRecordStopClicked();
    
    // 数据更新
    void onDataUpdated(const SensorData &data);
    
    // 加载历史数据
    void onLoadHistoryClicked();
    
    // 显示类型切换
    void onDisplayTypeChanged(int index);
    
    // 状态更新
    void onCollectorStarted();
    void onCollectorStopped();
    void onRecorderStarted(const QString &filePath);
    void onRecorderStopped();
    
    // 错误处理
    void onCollectorError(const QString &message);
    void onRecorderError(const QString &message);
    
    // 定时更新UI
    void onUpdateTimer();

private:
    // 初始化UI
    void initUI();
    
    // 连接信号槽
    void connectSignals();
    
    // 更新数据显示
    void updateDataDisplay(const SensorData &data);
    
    // 加载CSV文件
    bool loadCsvFile(const QString &filePath);

private:
    Ui::MainWindow *ui;
    
    // 核心组件
    DataModel *m_dataModel;
    DataCollector *m_dataCollector;
    DataRecorder *m_dataRecorder;
    ChartWidget *m_chartWidget;
    
    // 定时器
    QTimer *m_updateTimer;
    
    // 状态
    bool m_isCollecting;
    bool m_isRecording;
};

#endif // MAINWINDOW_H
