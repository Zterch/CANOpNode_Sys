#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <QObject>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include "datamodel.h"

/**
 * @brief 电机控制器 - 通过文件接口控制电机
 * 
 * 工作原理：
 * 1. 写入控制命令到配置文件
 * 2. 算法程序读取配置文件并执行命令
 * 3. 通过读取数据文件获取实时状态
 */
class MotorController : public QObject
{
    Q_OBJECT

public:
    explicit MotorController(QObject *parent = nullptr);
    ~MotorController();

    // 设置命令文件路径
    void setCommandFilePath(const QString &path) { m_commandFilePath = path; }
    QString commandFilePath() const { return m_commandFilePath; }
    
    // 发送速度命令 (rpm)
    bool setVelocity(double rpm);
    
    // 发送位置命令 (counts或圈数)
    bool setPosition(double position);
    
    // 停止电机
    bool stop();
    
    // 使能/失能电机
    bool enable(bool enable);

signals:
    // 命令发送结果
    void commandSent(bool success, const QString &message);
    
    // 错误信号
    void error(const QString &message);

private:
    // 写入命令到文件
    bool writeCommand(const QString &command);

private:
    QString m_commandFilePath;
};

#endif // MOTORCONTROLLER_H
