#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serial(nullptr)
    , updateTimer(nullptr)
    , currentVoltage(0.0f)
    , currentCurrent(0.0f)
    , targetVoltage(24.0f)
    , targetCurrent(0.44f)
    , waitingForVoltageResponse(true)
{
    ui->setupUi(this);
    
    // 初始化串口
    serial = new QSerialPort(this);
    
    // 初始化定时器，20Hz更新频率
    updateTimer = new QTimer(this);
    updateTimer->setInterval(50); // 50ms = 20Hz
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::onUpdateTimer);
    
    // 连接信号槽
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::onSerialDataReceived);
    
    // 初始化UI
    updateDisplay();
    
    // 扫描可用串口
    refreshSerialPorts();
}

MainWindow::~MainWindow()
{
    if (serial->isOpen()) {
        serial->close();
    }
    delete serial;
    delete updateTimer;
    delete ui;
}

void MainWindow::refreshSerialPorts()
{
    ui->comboBoxPort->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        ui->comboBoxPort->addItem(info.portName());
    }
}

void MainWindow::on_pushButtonConnect_clicked()
{
    if (serial->isOpen()) {
        disconnectSerial();
    } else {
        connectSerial();
    }
}

void MainWindow::connectSerial()
{
    serial->setPortName("/dev/"+ui->comboBoxPort->currentText());
    serial->setBaudRate(ui->comboBoxBaud->currentText().toInt());
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    
    if (serial->open(QIODevice::ReadWrite)) {
        log("串口连接成功: " + serial->portName());
        ui->pushButtonConnect->setText("断开");
        updateTimer->start();
    } else {
        QString errorStr = serial->errorString();
        log("串口连接失败: " + errorStr);
        if (errorStr.contains("Permission denied")) {
            log("权限错误: 请确保您有权限访问该串口");
            log("尝试运行: sudo usermod -a -G dialout $USER");
            log("然后重新登录系统");
        }
    }
}

void MainWindow::disconnectSerial()
{
    if (serial->isOpen()) {
        serial->close();
        updateTimer->stop();
        ui->pushButtonConnect->setText("连接");
        log("串口已断开");
    }
}

void MainWindow::onUpdateTimer()
{
    // 读取电压和电流
    if (serial->isOpen()) {
        if (waitingForVoltageResponse) {
            // 发送读取电压命令
            sendCommand(0x01, 0x09A0, 0x0002);
        } else {
            // 发送读取电流命令
            sendCommand(0x01, 0xA1A2, 0x0002);
        }
    }
}

void MainWindow::onSerialDataReceived()
{
    QByteArray data = serial->readAll();
    if (!data.isEmpty()) {
        // // 打印接收到的数据
        // QString hexData;
        // for (int i = 0; i < data.size(); i++) {
        //     hexData += QString("%1 ").arg(static_cast<unsigned char>(data[i]), 2, 16, QChar('0')).toUpper();
        // }
        // log("接收到数据: " + hexData);
        parseResponse(data);
    }
}

void MainWindow::sendCommand(uint8_t funcCode, uint16_t regAddr, uint16_t data)
{
    QByteArray cmd;
    cmd.append(static_cast<char>(0xAA)); // 设备地址
    cmd.append(static_cast<char>(funcCode));
    cmd.append(static_cast<char>((regAddr >> 8) & 0xFF));
    cmd.append(static_cast<char>(regAddr & 0xFF));
    if (funcCode == 0x06) {
        cmd.append(static_cast<char>((data >> 8) & 0xFF));
        cmd.append(static_cast<char>(data & 0xFF));
    } else if (funcCode == 0x01) {
        cmd.append(static_cast<char>(0x00));
        cmd.append(static_cast<char>(0x02));
    }
    
    // 计算CRC16
    uint16_t crc = calculateCRC(cmd);
    cmd.append(static_cast<char>(crc >> 8));
    cmd.append(static_cast<char>(crc & 0xFF));
    
    // // 打印发送的命令
    // QString hexCmd;
    // for (int i = 0; i < cmd.size(); i++) {
    //     hexCmd += QString("%1 ").arg(static_cast<unsigned char>(cmd[i]), 2, 16, QChar('0')).toUpper();
    // }
    // log("发送命令: " + hexCmd);
    
    serial->write(cmd);
}

void MainWindow::parseResponse(const QByteArray &data)
{
    // // 打印数据长度
    // log("响应数据长度: " + QString::number(data.size()));
    
    if (data.size() < 6) return;
    
    uint8_t addr = static_cast<uint8_t>(data[0]);
    uint8_t func = static_cast<uint8_t>(data[1]);
    
    if (addr == 0xAA) {
        if (func == 0x01) {
            // 读取数据响应
            if (data.size() >= 6) {
                uint16_t value = (static_cast<uint8_t>(data[3]) << 8) | static_cast<uint8_t>(data[4]);
                
                if (waitingForVoltageResponse) {
                    // 处理电压响应
                    currentVoltage = value / 1000.0f; // 转换为V
                    // log("电压值: " + QString::number(currentVoltage, 'f', 2) + " V");
                    waitingForVoltageResponse = false; // 下一次等待电流响应
                } else {
                    // 处理电流响应
                    currentCurrent = value / 1000.0f; // 转换为A
                    // log("电流值: " + QString::number(currentCurrent, 'f', 2) + " A");
                    waitingForVoltageResponse = true; // 下一次等待电压响应
                }
                
                updateDisplay();
            }
        } else if (func == 0x06) {
            // 设置数据响应
            // log("设置命令执行成功");
        }
    }
}

void MainWindow::updateDisplay()
{
    ui->labelVoltage->setText(QString::number(currentVoltage, 'f', 2) + " V");
    ui->labelCurrent->setText(QString::number(currentCurrent, 'f', 2) + " A");
    ui->labelTargetVoltage->setText(QString::number(targetVoltage, 'f', 2) + " V");
    ui->labelTargetCurrent->setText(QString::number(targetCurrent, 'f', 2) + " A");
}

void MainWindow::log(const QString &message)
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    ui->textEditLog->append(timestamp + " " + message);
}

uint16_t MainWindow::calculateCRC(const QByteArray &data)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < data.size(); i++) {
        crc ^= static_cast<uint8_t>(data[i]);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void MainWindow::on_pushButtonSetCurrent_clicked()
{
    bool ok;
    float value = ui->lineEditCurrent->text().toFloat(&ok);
    if (ok && value >= 0.0f && value <= 0.88f) {
        targetCurrent = value;
        // 发送设置电流命令
        if (serial->isOpen()) {
            uint16_t intValue = static_cast<uint16_t>(targetCurrent * 1000); // 转换为mA
            sendCommand(0x06, 0x0304, intValue);
            log("设置电流: " + QString::number(targetCurrent, 'f', 2) + " A");
            updateDisplay();
        }
    }
}

void MainWindow::on_pushButtonSetVoltage_clicked()
{
    bool ok;
    float value = ui->lineEditVoltage->text().toFloat(&ok);
    if (ok && value >= 0.0f && value <= 48.0f) {
        targetVoltage = value;
        // 发送设置电压命令
        if (serial->isOpen()) {
            uint16_t intValue = static_cast<uint16_t>(targetVoltage * 1000); // 转换为mV
            sendCommand(0x06, 0x0102, intValue);
            log("设置电压: " + QString::number(targetVoltage, 'f', 2) + " V");
            updateDisplay();
        }
    }
}

void MainWindow::on_pushButtonInc_clicked()
{
    if (targetCurrent < 0.88f) {
        targetCurrent += CURRENT_STEP;
        ui->lineEditCurrent->setText(QString::number(targetCurrent, 'f', 2));
    }
}

void MainWindow::on_pushButtonDec_clicked()
{
    if (targetCurrent > 0.0f) {
        targetCurrent -= CURRENT_STEP;
        ui->lineEditCurrent->setText(QString::number(targetCurrent, 'f', 2));
    }
}

void MainWindow::on_pushButtonIncVoltage_clicked()
{
    if (targetVoltage < 48.0f) {
        targetVoltage += VOLTAGE_STEP;
        ui->lineEditVoltage->setText(QString::number(targetVoltage, 'f', 2));
    }
}

void MainWindow::on_pushButtonDecVoltage_clicked()
{
    if (targetVoltage > 0.0f) {
        targetVoltage -= VOLTAGE_STEP;
        ui->lineEditVoltage->setText(QString::number(targetVoltage, 'f', 2));
    }
}

