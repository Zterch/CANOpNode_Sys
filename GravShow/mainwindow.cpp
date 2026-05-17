#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_isCollecting(false)
    , m_isRecording(false)
{
    ui->setupUi(this);
    
    // 初始化核心组件
    m_dataModel = new DataModel(this);
    m_dataCollector = new ShmDataCollector(m_dataModel, this);
    m_dataRecorder = new DataRecorder(m_dataModel, this);
    
    // 创建图表控件（传入数据模型）
    m_chartWidget = new ChartWidget(m_dataModel, this);
    
    // 初始化UI
    initUI();
    
    // 连接信号槽
    connectSignals();
    
    // 设置窗口标题
    setWindowTitle("重力卸载系统数据监控 - GravShow v2.0 [共享内存模式]");
    
    // 更新连接状态
    updateConnectionStatus();
    
    qDebug() << "MainWindow initialized (Shared Memory Mode)";
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initUI()
{
    // 将图表添加到chartContainer
    QVBoxLayout *chartLayout = new QVBoxLayout(ui->chartContainer);
    chartLayout->addWidget(m_chartWidget);
    chartLayout->setContentsMargins(0, 0, 0, 0);
    
    // 初始化按钮状态
    ui->btnStart->setEnabled(true);
    ui->btnStop->setEnabled(false);
    ui->btnRecordStart->setEnabled(false);
    ui->btnRecordStop->setEnabled(false);
    
    // 初始化状态标签
    ui->labelCollectStatus->setText("未连接");
    ui->labelCollectStatus->setStyleSheet("color: gray;");
    ui->labelRecordStatus->setText("未记录");
    ui->labelRecordStatus->setStyleSheet("color: gray;");
    
    // 隐藏数据文件路径选择（共享内存不需要）
    ui->lineEditDataPath->setText("共享内存: /gravshow_shm");
    ui->lineEditDataPath->setEnabled(false);
    ui->btnBrowse->setEnabled(false);
    
    // 添加显示类型选项
    ui->comboBoxDisplayType->addItem("全部显示", 0);
    ui->comboBoxDisplayType->addItem("仅绳长", 2);
    ui->comboBoxDisplayType->addItem("仅编码器", 1);
    ui->comboBoxDisplayType->addItem("仅电流", 3);
    ui->comboBoxDisplayType->addItem("仅电压", 4);
    ui->comboBoxDisplayType->addItem("仅电机速度", 5);
    ui->comboBoxDisplayType->addItem("仅电机位置", 6);
    ui->comboBoxDisplayType->addItem("仅压力", 7);
    
    // 创建定时器用于更新UI
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &MainWindow::onUpdateTimer);
    m_updateTimer->start(50);  // 50ms = 20Hz更新UI
}

void MainWindow::connectSignals()
{
    // 按钮信号
    connect(ui->btnStart, &QPushButton::clicked, this, &MainWindow::onStartClicked);
    connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(ui->btnRecordStart, &QPushButton::clicked, this, &MainWindow::onRecordStartClicked);
    connect(ui->btnRecordStop, &QPushButton::clicked, this, &MainWindow::onRecordStopClicked);
    connect(ui->btnLoadHistory, &QPushButton::clicked, this, &MainWindow::onLoadHistoryClicked);
    
    // 电机控制按钮（通过共享内存）
    connect(ui->btnSetVelocity, &QPushButton::clicked, this, &MainWindow::onSetVelocityClicked);
    connect(ui->btnSetPosition, &QPushButton::clicked, this, &MainWindow::onSetPositionClicked);
    connect(ui->btnMotorEnable, &QPushButton::clicked, this, &MainWindow::onMotorEnableClicked);
    connect(ui->btnMotorDisable, &QPushButton::clicked, this, &MainWindow::onMotorDisableClicked);
    connect(ui->btnMotorStop, &QPushButton::clicked, this, &MainWindow::onMotorStopClicked);
    
    // 电源板控制按钮（通过共享内存）
    connect(ui->btnSetPowerCurrent, &QPushButton::clicked, this, &MainWindow::onSetPowerCurrentClicked);
    connect(ui->btnSetPowerVoltage, &QPushButton::clicked, this, &MainWindow::onSetPowerVoltageClicked);
    connect(ui->btnPowerOn, &QPushButton::clicked, this, &MainWindow::onPowerOnClicked);
    connect(ui->btnPowerOff, &QPushButton::clicked, this, &MainWindow::onPowerOffClicked);
    
    // 显示类型切换
    connect(ui->comboBoxDisplayType, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onDisplayTypeChanged);
    
    // 数据采集器信号（共享内存）
    connect(m_dataCollector, &ShmDataCollector::started, this, &MainWindow::onCollectorStarted);
    connect(m_dataCollector, &ShmDataCollector::stopped, this, &MainWindow::onCollectorStopped);
    connect(m_dataCollector, &ShmDataCollector::connected, this, &MainWindow::onCollectorConnected);
    connect(m_dataCollector, &ShmDataCollector::disconnected, this, &MainWindow::onCollectorDisconnected);
    connect(m_dataCollector, &ShmDataCollector::error, this, &MainWindow::onCollectorError);
    
    // 数据记录器信号
    connect(m_dataRecorder, &DataRecorder::recordingStarted, this, &MainWindow::onRecorderStarted);
    connect(m_dataRecorder, &DataRecorder::recordingStopped, this, &MainWindow::onRecorderStopped);
    connect(m_dataRecorder, &DataRecorder::error, this, &MainWindow::onRecorderError);
    
    // 数据更新信号
    connect(m_dataModel, &DataModel::dataUpdated, this, &MainWindow::onDataUpdated);
}

void MainWindow::onStartClicked()
{
    // 开始采集（20Hz）
    m_dataCollector->start(50);  // 50ms = 20Hz
    
    ui->btnStart->setEnabled(false);
    ui->btnStop->setEnabled(true);
    ui->btnRecordStart->setEnabled(true);
    
    // 清空历史数据
    m_dataModel->clearHistory();
    m_chartWidget->clearData();
    
    m_isCollecting = true;
}

void MainWindow::onStopClicked()
{
    // 如果正在记录，先停止记录
    if (m_isRecording) {
        onRecordStopClicked();
    }
    
    // 停止采集
    m_dataCollector->stop();
    
    ui->btnStart->setEnabled(true);
    ui->btnStop->setEnabled(false);
    ui->btnRecordStart->setEnabled(false);
    ui->btnRecordStop->setEnabled(false);
    
    m_isCollecting = false;
}

void MainWindow::onRecordStartClicked()
{
    // 开始记录
    if (m_dataRecorder->startRecording()) {
        ui->btnRecordStart->setEnabled(false);
        ui->btnRecordStop->setEnabled(true);
        m_isRecording = true;
    }
}

void MainWindow::onRecordStopClicked()
{
    // 停止记录
    m_dataRecorder->stopRecording();
    
    ui->btnRecordStart->setEnabled(true);
    ui->btnRecordStop->setEnabled(false);
    m_isRecording = false;
}

void MainWindow::onDataUpdated(const SensorData &data)
{
    // 数据更新只添加到缓冲区，图表更新由定时器统一处理
    // 避免每次数据到来都触发重绘，减少CPU负担
}

void MainWindow::onLoadHistoryClicked()
{
    QString filePath = QFileDialog::getOpenFileName(this, "加载历史数据",
        "/home/zterch/VS_Project/Nimo_COp_Prj/GravShow/logdata",
        "CSV文件 (*.csv);;所有文件 (*)");
    
    if (!filePath.isEmpty()) {
        if (loadCsvFile(filePath)) {
            QMessageBox::information(this, "成功", "历史数据加载成功！");
        } else {
            QMessageBox::warning(this, "错误", "无法加载历史数据文件！");
        }
    }
}

void MainWindow::onDisplayTypeChanged(int index)
{
    int type = ui->comboBoxDisplayType->currentData().toInt();
    m_chartWidget->setDisplayType(type);
}

void MainWindow::onCollectorStarted()
{
    ui->labelCollectStatus->setText("采集中 (20Hz)");
    ui->labelCollectStatus->setStyleSheet("color: green;");
}

void MainWindow::onCollectorStopped()
{
    ui->labelCollectStatus->setText("已停止");
    ui->labelCollectStatus->setStyleSheet("color: red;");
}

void MainWindow::onCollectorConnected()
{
    updateConnectionStatus();
    ui->labelCollectStatus->setText("已连接");
    ui->labelCollectStatus->setStyleSheet("color: green;");
}

void MainWindow::onCollectorDisconnected()
{
    updateConnectionStatus();
    ui->labelCollectStatus->setText("断开");
    ui->labelCollectStatus->setStyleSheet("color: red;");
}

void MainWindow::onRecorderStarted(const QString &filePath)
{
    ui->labelRecordStatus->setText("记录中: " + QFileInfo(filePath).fileName());
    ui->labelRecordStatus->setStyleSheet("color: green;");
}

void MainWindow::onRecorderStopped()
{
    ui->labelRecordStatus->setText("未记录");
    ui->labelRecordStatus->setStyleSheet("color: gray;");
}

void MainWindow::onCollectorError(const QString &message)
{
    QMessageBox::warning(this, "采集错误", message);
}

void MainWindow::onRecorderError(const QString &message)
{
    QMessageBox::warning(this, "记录错误", message);
}

void MainWindow::onUpdateTimer()
{
    // 定期更新UI显示
    SensorData data = m_dataModel->latestData();
    updateDataDisplay(data);
    
    // 更新连接状态
    updateConnectionStatus();
}

void MainWindow::updateConnectionStatus()
{
    if (m_dataCollector->isConnected()) {
        if (!m_isCollecting) {
            ui->labelCollectStatus->setText("就绪");
            ui->labelCollectStatus->setStyleSheet("color: blue;");
        }
    } else {
        ui->labelCollectStatus->setText("未连接");
        ui->labelCollectStatus->setStyleSheet("color: gray;");
    }
}

void MainWindow::updateDataDisplay(const SensorData &data)
{
    // 更新数值显示
    ui->labelRopeValue->setText(QString::number(data.ropeLength, 'f', 3) + " m");
    ui->labelEncoderValue->setText(QString::number(data.encoderValue));
    ui->labelCurrentValue->setText(QString::number(data.current, 'f', 3) + " A");
    ui->labelVoltageValue->setText(QString::number(data.voltage, 'f', 2) + " V");
    ui->labelMotorSpeedValue->setText(QString::number(data.motorSpeed, 'f', 1) + " rpm");
    ui->labelMotorPositionValue->setText(QString::number(data.motorPosition, 'f', 0));
    ui->labelMotorStatusValue->setText(data.motorStatusStr.isEmpty() ? "Unknown" : data.motorStatusStr);
    ui->labelPressureValue->setText(QString::number(data.pressure, 'f', 3) + " kg");
    
    // 更新数据点数
    ui->labelDataCountValue->setText(QString::number(m_dataModel->dataCount()));
}

bool MainWindow::loadCsvFile(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }
    
    QTextStream in(&file);
    
    // 跳过头部
    if (!in.atEnd()) {
        in.readLine();
    }
    
    // 清空现有数据
    m_dataModel->clearHistory();
    QVector<SensorData> history;
    
    // 读取数据
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList parts = line.split(",");
        
        if (parts.size() >= 5) {
            SensorData data;
            data.timestamp = QDateTime::fromString(parts[0], "yyyy-MM-dd hh:mm:ss.zzz");
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
            
            history.append(data);
        }
    }
    
    file.close();
    
    // 加载到图表
    if (!history.isEmpty()) {
        m_chartWidget->loadFromHistory(history);
        return true;
    }
    
    return false;
}

// 电机控制槽函数（通过共享内存）
void MainWindow::onSetVelocityClicked()
{
    double velocity = ui->spinBoxVelocity->value();
    if (m_dataCollector->sendMotorCommand(1, velocity, 0)) {
        QMessageBox::information(this, "电机控制", 
            QString("速度命令已发送: %1 rpm").arg(velocity));
    } else {
        QMessageBox::warning(this, "电机控制错误", 
            "发送速度命令失败，请检查共享内存连接");
    }
}

void MainWindow::onSetPositionClicked()
{
    double position = ui->spinBoxPosition->value();
    if (m_dataCollector->sendMotorCommand(2, position, 0)) {
        QMessageBox::information(this, "电机控制", 
            QString("位置命令已发送: %1").arg(position));
    } else {
        QMessageBox::warning(this, "电机控制错误", 
            "发送位置命令失败，请检查共享内存连接");
    }
}

void MainWindow::onMotorEnableClicked()
{
    if (m_dataCollector->sendMotorCommand(4, 0, 0)) {
        QMessageBox::information(this, "电机控制", "电机使能命令已发送");
    } else {
        QMessageBox::warning(this, "电机控制错误", "发送使能命令失败");
    }
}

void MainWindow::onMotorDisableClicked()
{
    if (m_dataCollector->sendMotorCommand(5, 0, 0)) {
        QMessageBox::information(this, "电机控制", "电机失能命令已发送");
    } else {
        QMessageBox::warning(this, "电机控制错误", "发送失能命令失败");
    }
}

void MainWindow::onMotorStopClicked()
{
    if (m_dataCollector->sendMotorCommand(3, 0, 0)) {
        QMessageBox::information(this, "电机控制", "电机停止命令已发送");
    } else {
        QMessageBox::warning(this, "电机控制错误", "发送停止命令失败");
    }
}

// 算法控制槽函数
void MainWindow::onAlgorithmStartClicked()
{
    if (m_dataCollector->sendAlgorithmStart()) {
        QMessageBox::information(this, "算法控制", "算法启动命令已发送");
    } else {
        QMessageBox::warning(this, "算法控制错误", "发送启动命令失败");
    }
}

void MainWindow::onAlgorithmStopClicked()
{
    if (m_dataCollector->sendAlgorithmStop()) {
        QMessageBox::information(this, "算法控制", "算法停止命令已发送");
    } else {
        QMessageBox::warning(this, "算法控制错误", "发送停止命令失败");
    }
}

// 电源板控制槽函数
void MainWindow::onSetPowerCurrentClicked()
{
    double current = ui->spinBoxPowerCurrent->value();
    if (m_dataCollector->sendPowerCommand(6, current)) {  // cmd_type 6 = 设置电流
        QMessageBox::information(this, "电源板控制", 
            QString("电流命令已发送: %1 A").arg(current));
    } else {
        QMessageBox::warning(this, "电源板控制错误", 
            "发送电流命令失败，请检查共享内存连接");
    }
}

void MainWindow::onSetPowerVoltageClicked()
{
    double voltage = ui->spinBoxPowerVoltage->value();
    if (m_dataCollector->sendPowerCommand(7, voltage)) {  // cmd_type 7 = 设置电压
        QMessageBox::information(this, "电源板控制", 
            QString("电压命令已发送: %1 V").arg(voltage));
    } else {
        QMessageBox::warning(this, "电源板控制错误", 
            "发送电压命令失败，请检查共享内存连接");
    }
}

void MainWindow::onPowerOnClicked()
{
    // 电源开启：设置默认电流0.5A
    if (m_dataCollector->sendPowerCommand(6, 0.5)) {
        QMessageBox::information(this, "电源板控制", "电源已开启 (0.5A)");
    } else {
        QMessageBox::warning(this, "电源板控制错误", "开启电源失败");
    }
}

void MainWindow::onPowerOffClicked()
{
    // 电源关闭：设置最小电流50mA
    if (m_dataCollector->sendPowerCommand(6, 0.05)) {
        QMessageBox::information(this, "电源板控制", "电源已关闭 (50mA)");
    } else {
        QMessageBox::warning(this, "电源板控制错误", "关闭电源失败");
    }
}
