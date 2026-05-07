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
    m_dataCollector = new DataCollector(m_dataModel, this);
    m_dataRecorder = new DataRecorder(m_dataModel, this);
    
    // 创建图表控件
    m_chartWidget = new ChartWidget(this);
    
    // 初始化UI
    initUI();
    
    // 连接信号槽
    connectSignals();
    
    // 设置窗口标题
    setWindowTitle("重力卸载系统数据监控 - GravShow v1.0");
    
    qDebug() << "MainWindow initialized";
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
    
    // 设置数据文件路径
    ui->lineEditDataPath->setText(m_dataCollector->dataFilePath());
    
    // 添加显示类型选项
    ui->comboBoxDisplayType->addItem("全部显示", 0);
    ui->comboBoxDisplayType->addItem("仅压力", 1);
    ui->comboBoxDisplayType->addItem("仅绳长", 2);
    ui->comboBoxDisplayType->addItem("仅电流", 3);
    ui->comboBoxDisplayType->addItem("仅电压", 4);
    
    // 创建定时器用于更新UI
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &MainWindow::onUpdateTimer);
    m_updateTimer->start(100);  // 100ms更新一次UI
}

void MainWindow::connectSignals()
{
    // 按钮信号
    connect(ui->btnStart, &QPushButton::clicked, this, &MainWindow::onStartClicked);
    connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(ui->btnRecordStart, &QPushButton::clicked, this, &MainWindow::onRecordStartClicked);
    connect(ui->btnRecordStop, &QPushButton::clicked, this, &MainWindow::onRecordStopClicked);
    connect(ui->btnLoadHistory, &QPushButton::clicked, this, &MainWindow::onLoadHistoryClicked);
    connect(ui->btnBrowse, &QPushButton::clicked, this, [this]() {
        QString filePath = QFileDialog::getOpenFileName(this, "选择数据文件", 
            "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/share", "文本文件 (*.txt)");
        if (!filePath.isEmpty()) {
            ui->lineEditDataPath->setText(filePath);
            m_dataCollector->setDataFilePath(filePath);
        }
    });
    
    // 显示类型切换
    connect(ui->comboBoxDisplayType, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onDisplayTypeChanged);
    
    // 数据采集器信号
    connect(m_dataCollector, &DataCollector::started, this, &MainWindow::onCollectorStarted);
    connect(m_dataCollector, &DataCollector::stopped, this, &MainWindow::onCollectorStopped);
    connect(m_dataCollector, &DataCollector::error, this, &MainWindow::onCollectorError);
    
    // 数据记录器信号
    connect(m_dataRecorder, &DataRecorder::recordingStarted, this, &MainWindow::onRecorderStarted);
    connect(m_dataRecorder, &DataRecorder::recordingStopped, this, &MainWindow::onRecorderStopped);
    connect(m_dataRecorder, &DataRecorder::error, this, &MainWindow::onRecorderError);
    
    // 数据更新信号
    connect(m_dataModel, &DataModel::dataUpdated, this, &MainWindow::onDataUpdated);
}

void MainWindow::onStartClicked()
{
    // 设置数据文件路径
    m_dataCollector->setDataFilePath(ui->lineEditDataPath->text());
    
    // 开始采集
    m_dataCollector->start(100);  // 100ms采集间隔
    
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
    // 更新图表
    m_chartWidget->addDataPoint(data);
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
    ui->labelCollectStatus->setText("采集中");
    ui->labelCollectStatus->setStyleSheet("color: green;");
}

void MainWindow::onCollectorStopped()
{
    ui->labelCollectStatus->setText("已停止");
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
}

void MainWindow::updateDataDisplay(const SensorData &data)
{
    // 更新数值显示
    ui->labelCurrentValue->setText(QString::number(data.current, 'f', 3) + " A");
    ui->labelVoltageValue->setText(QString::number(data.voltage, 'f', 2) + " V");
    ui->labelPressureValue->setText(QString::number(data.pressure, 'f', 3) + " kg");
    ui->labelRopeValue->setText(QString::number(data.ropeLength, 'f', 3) + " m");
    ui->labelSpeedValue->setText(QString::number(data.motorSpeed, 'f', 1) + " rpm");
    
    // 更新数据点数
    ui->labelDataCount->setText(QString::number(m_dataModel->dataCount()));
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
            if (parts.size() >= 6) {
                data.motorSpeed = parts[5].toDouble();
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
