#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QDateTime>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButtonConnect_clicked();
    void on_pushButtonSetCurrent_clicked();
    void on_pushButtonSetVoltage_clicked();
    void on_pushButtonInc_clicked();
    void on_pushButtonDec_clicked();
    void on_pushButtonIncVoltage_clicked();
    void on_pushButtonDecVoltage_clicked();
    
    void onSerialDataReceived();
    void onUpdateTimer();

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    QTimer *updateTimer;
    
    // 数据
    float currentVoltage;
    float currentCurrent;
    float targetVoltage;
    float targetCurrent;
    
    // 状态
    bool waitingForVoltageResponse; // 标记是否在等待电压响应
    
    // 常量
    const float CURRENT_STEP = 0.01f; // 0.01A步长
    const float VOLTAGE_STEP = 0.1f;  // 0.1V步长
    
    // 方法
    void refreshSerialPorts();
    void connectSerial();
    void disconnectSerial();
    void sendCommand(uint8_t funcCode, uint16_t regAddr, uint16_t data);
    void parseResponse(const QByteArray &data);
    void updateDisplay();
    void log(const QString &message);
    uint16_t calculateCRC(const QByteArray &data);
};

#endif // MAINWINDOW_H
