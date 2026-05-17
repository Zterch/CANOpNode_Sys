#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "datamodel.h"
#include "shmdatacollector.h"
#include "datarecorder.h"
#include "chartwidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/**
 * @brief 重力卸载系统数据监控主窗口 v2.0
 * 
 * 新特性：
 * - 共享内存通信（20Hz）
 * - 算法独立监控
 * - 远程电机控制
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
    void onCollectorConnected();
    void onCollectorDisconnected();
    void onRecorderStarted(const QString &filePath);
    void onRecorderStopped();
    
    // 错误处理
    void onCollectorError(const QString &message);
    void onRecorderError(const QString &message);
    
    // 定时更新UI
    void onUpdateTimer();
    
    // 电机控制槽函数（通过共享内存）
    void onSetVelocityClicked();
    void onSetPositionClicked();
    void onMotorEnableClicked();
    void onMotorDisableClicked();
    void onMotorStopClicked();
    
    // 算法控制
    void onAlgorithmStartClicked();
    void onAlgorithmStopClicked();
    
    // 电源板控制槽函数
    void onSetPowerCurrentClicked();
    void onSetPowerVoltageClicked();
    void onPowerOnClicked();
    void onPowerOffClicked();

private:
    // 初始化UI
    void initUI();
    
    // 连接信号槽
    void connectSignals();
    
    // 更新数据显示
    void updateDataDisplay(const SensorData &data);
    
    // 更新连接状态显示
    void updateConnectionStatus();
    
    // 加载CSV文件
    bool loadCsvFile(const QString &filePath);

private:
    Ui::MainWindow *ui;
    
    // 核心组件
    DataModel *m_dataModel;
    ShmDataCollector *m_dataCollector;  // 使用共享内存收集器
    DataRecorder *m_dataRecorder;
    ChartWidget *m_chartWidget;
    
    // 定时器
    QTimer *m_updateTimer;
    
    // 状态
    bool m_isCollecting;
    bool m_isRecording;
};

#endif // MAINWINDOW_H
