#include "motorcontroller.h"
#include <QDebug>

MotorController::MotorController(QObject *parent)
    : QObject(parent)
    , m_commandFilePath("/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/share/motor_command.txt")
{
}

MotorController::~MotorController()
{
}

bool MotorController::setVelocity(double rpm)
{
    QString command = QString("VELOCITY:%1").arg(rpm, 0, 'f', 2);
    bool success = writeCommand(command);
    
    if (success) {
        emit commandSent(true, QString("速度命令已发送: %1 rpm").arg(rpm));
    } else {
        emit error("发送速度命令失败");
    }
    
    return success;
}

bool MotorController::setPosition(double position)
{
    QString command = QString("POSITION:%1").arg(position, 0, 'f', 2);
    bool success = writeCommand(command);
    
    if (success) {
        emit commandSent(true, QString("位置命令已发送: %1").arg(position));
    } else {
        emit error("发送位置命令失败");
    }
    
    return success;
}

bool MotorController::stop()
{
    bool success = writeCommand("STOP");
    
    if (success) {
        emit commandSent(true, "停止命令已发送");
    } else {
        emit error("发送停止命令失败");
    }
    
    return success;
}

bool MotorController::enable(bool enable)
{
    QString command = enable ? "ENABLE" : "DISABLE";
    bool success = writeCommand(command);
    
    if (success) {
        emit commandSent(true, enable ? "电机使能命令已发送" : "电机失能命令已发送");
    } else {
        emit error("发送使能命令失败");
    }
    
    return success;
}

bool MotorController::writeCommand(const QString &command)
{
    QFile file(m_commandFilePath);
    
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        qDebug() << "无法打开命令文件:" << m_commandFilePath;
        return false;
    }
    
    QTextStream out(&file);
    out << command << "\n";
    out << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz") << "\n";
    file.close();
    
    qDebug() << "Motor command written:" << command;
    return true;
}
