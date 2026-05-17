#include "shmdatacollector.h"
#include "datamodel.h"
#include <QDebug>
#include <cstring>
#include <errno.h>

ShmDataCollector::ShmDataCollector(DataModel *model, QObject *parent)
    : QObject(parent)
    , m_model(model)
    , m_shmFd(-1)
    , m_shmPtr(nullptr)
    , m_dataPtr(nullptr)
    , m_commandPtr(nullptr)
    , m_shmConnected(false)
    , m_lastSequence(0)
    , m_packetLossCount(0)
    , m_syncCount(0)
    , m_commandId(0)
{
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &ShmDataCollector::collectData);
    
    // 尝试初始化共享内存
    initSharedMemory();
}

ShmDataCollector::~ShmDataCollector()
{
    stop();
    closeSharedMemory();
}

bool ShmDataCollector::initSharedMemory()
{
    // 打开共享内存（已存在，由下位机创建）
    m_shmFd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (m_shmFd < 0) {
        if (errno == ENOENT) {
            qDebug() << "Shared memory not found, waiting for server...";
        } else if (errno == EACCES) {
            qDebug() << "Permission denied, trying to fix...";
            // 尝试用chmod修复权限
            int ret = system("sudo chmod 666 /dev/shm/gravshow_shm 2>/dev/null || chmod 666 /dev/shm/gravshow_shm 2>/dev/null");
            (void)ret;
            // 再次尝试打开
            m_shmFd = shm_open(SHM_NAME, O_RDWR, 0666);
            if (m_shmFd < 0) {
                qDebug() << "shm_open still failed after chmod:" << strerror(errno);
                emit error(QString("无法打开共享内存（权限问题）: %1\n请运行: sudo chmod 666 /dev/shm/gravshow_shm").arg(strerror(errno)));
                return false;
            }
            qDebug() << "Permission fixed, shared memory opened successfully";
        } else {
            qDebug() << "shm_open failed:" << strerror(errno);
            emit error(QString("无法打开共享内存: %1").arg(strerror(errno)));
        }
        return false;
    }
    
    // 映射共享内存
    m_shmPtr = mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, m_shmFd, 0);
    if (m_shmPtr == MAP_FAILED) {
        qDebug() << "mmap failed:" << strerror(errno);
        close(m_shmFd);
        m_shmFd = -1;
        emit error(QString("映射共享内存失败: %1").arg(strerror(errno)));
        return false;
    }
    
    // 设置指针
    m_dataPtr = reinterpret_cast<SharedData_t *>(static_cast<char *>(m_shmPtr) + DATA_OFFSET);
    m_commandPtr = reinterpret_cast<SharedCommand_t *>(static_cast<char *>(m_shmPtr) + COMMAND_OFFSET);
    
    m_shmConnected = true;
    emit connected();
    qDebug() << "Shared memory connected successfully";
    
    return true;
}

void ShmDataCollector::closeSharedMemory()
{
    if (m_shmPtr && m_shmPtr != MAP_FAILED) {
        munmap(m_shmPtr, SHM_SIZE);
        m_shmPtr = nullptr;
    }
    
    if (m_shmFd >= 0) {
        close(m_shmFd);
        m_shmFd = -1;
    }
    
    m_dataPtr = nullptr;
    m_commandPtr = nullptr;
    m_shmConnected = false;
    
    emit disconnected();
    qDebug() << "Shared memory disconnected";
}

void ShmDataCollector::start(int intervalMs)
{
    // 如果共享内存未连接，尝试重新连接
    if (!m_shmConnected) {
        if (!initSharedMemory()) {
            emit error("共享内存未就绪，请确保下位机程序已启动");
            return;
        }
    }
    
    if (!m_timer->isActive()) {
        m_timer->start(intervalMs);
        emit started();
        qDebug() << "ShmDataCollector started, interval:" << intervalMs << "ms (" 
                 << (1000.0 / intervalMs) << "Hz)";
    }
}

void ShmDataCollector::stop()
{
    if (m_timer->isActive()) {
        m_timer->stop();
        emit stopped();
        qDebug() << "ShmDataCollector stopped";
    }
}

void ShmDataCollector::setInterval(int ms)
{
    m_timer->setInterval(ms);
}

void ShmDataCollector::collectData()
{
    static int collectCount = 0;
    static QDateTime lastDebugTime = QDateTime::currentDateTime();
    
    // 每100次采集打印一次调试信息
    if (++collectCount % 100 == 0) {
        double elapsed = lastDebugTime.msecsTo(QDateTime::currentDateTime()) / 1000.0;
        qDebug() << "CollectData called" << collectCount << "times in" << elapsed << "seconds (" 
                 << (100.0 / elapsed) << "Hz)";
        lastDebugTime = QDateTime::currentDateTime();
    }
    
    // 如果未连接，尝试重新连接
    if (!m_shmConnected) {
        if (initSharedMemory()) {
            qDebug() << "Reconnected to shared memory";
            // 重置丢包检测状态
            m_lastSequence = 0;
            m_syncCount = 0;
        } else {
            return;  // 仍然无法连接，等待下次尝试
        }
    }
    
    SharedData_t data;
    if (readSharedData(&data)) {
        // 每100次成功读取打印一次调试信息
        if (collectCount % 100 == 0) {
            qDebug() << "Data received: sequence=" << data.sequence 
                     << "pressure=" << data.pressure_kg
                     << "motor_speed=" << data.motor_speed_rpm;
        }
        
        // 检查丢包 - 正确处理32位序列号回绕
        // 前3个数据包用于同步，不检测丢包（给系统时间对齐）
        if (m_lastSequence != 0 && m_syncCount >= 3) {
            // 使用有符号64位计算差值，处理UINT32回绕
            int64_t diff = (int64_t)data.sequence - (int64_t)m_lastSequence;
            
            // 处理回绕：如果差值超过INT32_MAX，说明发生了回绕
            if (diff > INT32_MAX) {
                diff -= 4294967296LL;  // 2^32
            } else if (diff < INT32_MIN) {
                diff += 4294967296LL;  // 2^32
            }
            
            // 正常情况下diff应该是1（每次递增1）
            // 允许2-5个包的跳跃（可能是正常的时序波动）
            if (diff > 1 && diff <= 5) {
                // 轻微丢包，静默记录
                m_packetLossCount += (int)(diff - 1);
            } else if (diff > 5) {
                // 明显丢包，打印警告
                int lost = (int)(diff - 1);
                m_packetLossCount += lost;
                qDebug() << "Packet loss detected:" << lost << "packets, total lost:" << m_packetLossCount;
            } else if (diff < 0) {
                // 序列号回绕或数据陈旧，重置计数器
                m_packetLossCount = 0;
                qDebug() << "Sequence number wrapped, resetting loss counter";
            }
        } else {
            m_syncCount++;
        }
        m_lastSequence = data.sequence;
        
        // 转换为SensorData并更新模型
        SensorData sensorData;
        convertToSensorData(data, sensorData);
        m_model->updateData(sensorData);
        
        // 发送数据接收信号
        emit dataReceived(data.sequence, data.timestamp_us);
    }
}

bool ShmDataCollector::readSharedData(SharedData_t *data)
{
    if (!m_shmConnected || !m_dataPtr || !data) {
        return false;
    }
    
    // 复制数据（使用内存屏障确保一致性）
    __sync_synchronize();
    memcpy(data, m_dataPtr, sizeof(SharedData_t));
    __sync_synchronize();
    
    // 验证魔数
    if (data->magic != SHM_MAGIC) {
        qDebug() << "Invalid magic number:" << QString::number(data->magic, 16);
        return false;
    }
    
    // 验证版本
    if (data->version != DATA_VERSION) {
        qDebug() << "Version mismatch, expected:" << DATA_VERSION << "got:" << data->version;
        return false;
    }
    
    return true;
}

bool ShmDataCollector::writeCommand(SharedCommand_t *cmd)
{
    if (!m_shmConnected || !m_commandPtr || !cmd) {
        return false;
    }
    
    // 设置魔数
    cmd->magic = SHM_MAGIC;
    
    // 递增命令ID
    cmd->command_id = ++m_commandId;
    
    // 写入命令（使用内存屏障）
    __sync_synchronize();
    memcpy(m_commandPtr, cmd, sizeof(SharedCommand_t));
    __sync_synchronize();
    
    return true;
}

bool ShmDataCollector::sendMotorCommand(int cmdType, double value, double accel)
{
    if (!m_shmConnected) {
        emit error("共享内存未连接，无法发送命令");
        return false;
    }
    
    SharedCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    
    cmd.cmd_type = cmdType;
    cmd.cmd_value = static_cast<float>(value);
    cmd.cmd_accel = static_cast<float>(accel);
    cmd.algorithm_start = false;
    cmd.algorithm_stop = false;
    
    if (writeCommand(&cmd)) {
        qDebug() << "Motor command sent: type=" << cmdType << "value=" << value;
        return true;
    }
    
    return false;
}

bool ShmDataCollector::sendAlgorithmStart()
{
    if (!m_shmConnected) {
        emit error("共享内存未连接，无法发送命令");
        return false;
    }
    
    SharedCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    
    cmd.cmd_type = 0;
    cmd.cmd_value = 0;
    cmd.cmd_accel = 0;
    cmd.algorithm_start = true;
    cmd.algorithm_stop = false;
    
    if (writeCommand(&cmd)) {
        qDebug() << "Algorithm start command sent";
        return true;
    }
    
    return false;
}

bool ShmDataCollector::sendAlgorithmStop()
{
    if (!m_shmConnected) {
        emit error("共享内存未连接，无法发送命令");
        return false;
    }
    
    SharedCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    
    cmd.cmd_type = 0;
    cmd.cmd_value = 0;
    cmd.cmd_accel = 0;
    cmd.algorithm_start = false;
    cmd.algorithm_stop = true;
    
    if (writeCommand(&cmd)) {
        qDebug() << "Algorithm stop command sent";
        return true;
    }
    
    return false;
}

bool ShmDataCollector::sendPowerCommand(int cmdType, double value)
{
    if (!m_shmConnected) {
        emit error("共享内存未连接，无法发送命令");
        return false;
    }
    
    SharedCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    
    cmd.cmd_type = cmdType;
    cmd.cmd_value = static_cast<float>(value);
    cmd.cmd_accel = 0;
    cmd.algorithm_start = false;
    cmd.algorithm_stop = false;
    
    if (writeCommand(&cmd)) {
        qDebug() << "Power command sent: type=" << cmdType << "value=" << value;
        return true;
    }
    
    return false;
}

void ShmDataCollector::convertToSensorData(const SharedData_t &sharedData, SensorData &sensorData)
{
    sensorData.timestamp = QDateTime::currentDateTime();
    
    // 传感器数据
    sensorData.pressure = sharedData.pressure_kg;
    sensorData.ropeLength = sharedData.rope_length_m;
    sensorData.encoderValue = sharedData.encoder_value;
    sensorData.encoderAngle = sharedData.encoder_angle_deg;
    
    // 电源数据
    sensorData.current = sharedData.current_a;
    sensorData.voltage = sharedData.voltage_v;
    
    // 电机数据
    sensorData.motorSpeed = sharedData.motor_speed_rpm;
    sensorData.motorPosition = sharedData.motor_position;
    sensorData.motorStatus = sharedData.motor_status;
    
    // 算法状态
    sensorData.algorithmState = sharedData.algorithm_state;
    sensorData.algorithmError = sharedData.algorithm_error;
    sensorData.emergencyStop = sharedData.emergency_stop;
    
    // 设置电机状态字符串
    switch (sharedData.motor_status) {
        case 1:
            sensorData.motorStatusStr = "Running";
            break;
        case 0:
            sensorData.motorStatusStr = "Stopped";
            break;
        default:
            sensorData.motorStatusStr = "Unknown";
            break;
    }
}
