#ifndef NIMSERVOSDK_H
#define NIMSERVOSDK_H

#if defined(_WIN32) || defined(__CYGWIN__) || defined(_WIN32_WCE)
    #if defined(NIMSERVOSDK_LIBRARY)
        #  define NIMSERVOSDKSHARED_EXPORT __declspec(dllexport)
    #else
        #  define NIMSERVOSDKSHARED_EXPORT __declspec(dllimport)
    #endif

    #define NIMAPI __cdecl

#else
    #define NIMSERVOSDKSHARED_EXPORT
    #define NIMAPI
#endif

#ifdef __cplusplus
extern "C"
{
#endif

enum CanBaudRate
{
    CAN_BT_10K = 0,
    CAN_BT_20K = 1,
    CAN_BT_50K = 2,
    CAN_BT_100K= 3,
    CAN_BT_125K= 4,
    CAN_BT_250K= 5,
    CAN_BT_500K= 6,
    CAN_BT_800K= 7,
    CAN_BT_1000K=8
};

enum ServoWorkMode
{
    SERVO_PP_MODE = 1,  //轮廓位置模式     PP
    SERVO_VM_MODE = 2,  //速度模式         VM
    SERVO_PV_MODE = 3,  //轮廓速度模式     PV
    SERVO_PT_MODE = 4,  //轮廓转矩模式     PT
    SERVO_HM_MODE = 6,  //原点回归模式     HM
    SERVO_IP_MODE = 7,  //位置插补模式     IP
    SERVO_CSP_MODE= 8,  //循环同步位置模式  CSP
    SERVO_CSV_MODE= 9,  //循环同步速度模式  CSV
    SERVO_CST_MODE= 10  //循环同步转矩模式  CST
};

enum ServoSDK_Error
{
    ServoSDK_NoError = 0,           // 0没有错误
    ServoSDK_NotRegisted,           // 1SDK没有注册
    ServoSDK_NotInitialized,        // 2SDK没有初始化
    ServoSDK_UnsupportedCommType,   // 3不支持的通信方式
    ServoSDK_ParamError,            // 4输入参数错误
    ServoSDK_CreateMasterFailed,    // 5创建主站失败
    ServoSDK_MasterNotExist,        // 6主站不存在
    ServoSDK_MasterStartFailed,     // 7主站启动失败
    ServoSDK_MasterNotRunning,      // 8主站未运行
    ServoSDK_SlaveNotOnline,        // 9从站不在线
    ServoSDK_LoadParamSheetFailed,  //10加载参数表错误
    ServoSDK_ParamNotExist,         //11请求的参数不存在
    ServoSDK_ReadSDOFailed,         //12读SDO失败
    ServoSDK_WriteSDOFailed,        //13写SDO失败
    ServoSDK_OperationNotAllowed,   //14操作不允许
    ServoSDK_MasterInternalError,   //15主站内部错误
    ServoSDK_SlaveInternalError,    //16从站内部错误
    ServoSDK_Cia402ModeError,       //17从站402模式错误
    ServoSDK_ReadWorkModeFailed,    //18读取工作模式失败
    ServoSDK_ReadStatusWordFailed,  //19读取状态字失败
    ServoSDK_ReadCurrentPosFailed,  //20读取当前位置失败
    ServoSDK_ReadRPDOConfigFailed,  //21读取PDO配置失败
    ServoSDK_ReadTPDOConfigFailed,  //22读取PDO配置失败
    ServoSDK_WriteControlWordFailed,//23写控制字失败
    ServoSDK_WriteTargetPosFailed,  //24写目标位置失败
    ServoSDK_WriteTargetVelFailed,  //25写目标速度失败
    ServoSDK_WriteGoHomeTypeFailed, //26写原点回归方式失败
    ServoSDK_GetHostInfoFailed,     //27获取主机信息失败
    ServoSDK_SaveParamsFailed,      //28保存参数失败
    ServoSDK_NoAvailableDevice,     //29没有可用的设备
    ServoSDK_Unknown = 255          //255未知错误
};

/**
 * @brief SDK初始化
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_init(const char* strSdkPath);

/**
 * @brief SDK 反初始化
 */
NIMSERVOSDKSHARED_EXPORT
void NIMAPI Nim_clean();

/**
 * @brief 设置日志输出标志
 * @param bit0=1输出到控制台；bit1=1输出到文件
 */
NIMSERVOSDKSHARED_EXPORT
void NIMAPI Nim_setLogFlags(int nFlags);

/**
 * @brief 获取日志输出标志
 * @return bit0=1输出到控制台；bit1=1输出到文件
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_getLogFlags();

/**
 * @brief 创建主站对象
 * @param nCommType 通信方式：0 CANopen；1 EtherCAT; 2 Modbus
 * @param handle 输出参数，成功时返回创建的主站对象句柄
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_create_master(int nCommType, unsigned int* handle);

/**
 * @brief 销毁主站对象
 * @param handle 由Nim_create_master函数创建的主站对象句柄
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_destroy_master(unsigned int handle);

/**
 * @brief 启动通信主站
 * @param hMaster 主站对象句柄
 * @param conn_str 连接字符串，json格式，具体内容参见使用手册
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_master_run(unsigned int hMaster, const char* conn_str);

/**
 * @brief 关闭通信主站
 * @param hMaster 主站对象句柄
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_master_stop(unsigned int hMaster);

/**
 * @brief 主站进入PreOP模式
 * @param hMaster 主站对象句柄
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_master_changeToPreOP(unsigned int hMaster);


/**
 * @brief 主站进入OP模式
 * @param hMaster 主站对象句柄
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_master_changeToOP(unsigned int hMaster);

/**
 * @brief 按照指定的地址范围扫描从站是否在线
 * @param hMaster 主站对象句柄
 * @param from 起始地址
 * @param to 结束地址
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_scan_nodes(unsigned int hMaster, int from, int to);

/**
 * @brief 查询从站是否在线
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @return 非零 在线；0 不在线
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_is_online(unsigned int hMaster, int nodeId);

/**
 * @brief 读取从站PDO配置
 * @param hMaster 主站对象句柄
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_read_PDOConfig(unsigned int hMaster, int nodeId);

/**
 * @brief 加载电机参数表
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param db_name 参数表数据库文件名
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_load_params(unsigned int hMaster, int nodeId, const char* db_name);

/**
 * @brief 读取从站参数
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param pParamNO 参数编号
 * @param puiValue 参数值
 * @param bSDO 1 使用SDO读；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_param_value(unsigned int hMaster, int nodeId, const char* pParamNO, unsigned int *puiValue, int bSDO);

/**
 * @brief 设置从站参数
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param pParamNO 参数编号
 * @param uiValue 参数值
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_param_value(unsigned int hMaster, int nodeId, const char* pParamNO, unsigned int uiValue, int bSDO);

/**
 * @brief 电机抱机
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_power_on(unsigned int hMaster, int nodeId, int bSDO);

/**
 * @brief 电机脱机
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return true 成功；false 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_power_off(unsigned int hMaster, int nodeId, int bSDO);

/**
 * @brief 设置控制字(6040)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param cw 控制字
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_controlWord(unsigned int hMaster, int nodeId, unsigned short cw, int bSDO);

/**
 * @brief 获取电机状态字(6041)
 * @param hMaster 主站对象句柄
 * @param nodeId 电机地址
 * @param sw 输出参数，成功时返回电机状态字
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_statusWord(unsigned int hMaster, int nodeId, unsigned short *sw, int bSDO);

/**
 * @brief 设置电机工作模式（6060,在脱机状态下设置）
 * @param hMaster 主站对象句柄
 * @param nodeId 电机地址
 * @param mode 模式
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_workMode(unsigned int hMaster, int nodeId, int mode, int bSDO);

/**
 * @brief 获取电机工作模式显示值(6061)
 * @param hMaster 主站对象句柄
 * @param nodeId 电机地址
 * @param mode 输出参数，成功时返回电机工作模式
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_workModeDisplay(unsigned int hMaster, int nodeId, int* mode, int bSDO);

/**
 * @brief 设置原点回归方式
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param type 回原点方式
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_homeType(unsigned int hMaster, int nodeId, unsigned char type);

/**
 * @brief 获取原点回归方式
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param type 回原点方式
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_homeType(unsigned int hMaster, int nodeId, unsigned char* type);

/**
 * @brief 原点回归
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param type 回原点方式
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_goHome(unsigned int hMaster, int nodeId, int bSDO);

/**
 * @brief 轮廓速度模式下正转
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param fVelocity 速度（用户单位/s）
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_forward(unsigned int hMaster, int nodeId, double fVelocity, int bSDO);

/**
 * @brief 轮廓速度模式下反转
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param fVelocity 速度（用户单位/s）
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_backward(unsigned int hMaster, int nodeId, double fVelocity, int bSDO);

/**
 * @brief 设置目标速度(60FF)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param fVelocity 目标速度（用户单位/s）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_targetVelocity(unsigned int hMaster, int nodeId, double fVelocity, int bSDO);

/**
 * @brief 设置VM模式下的目标速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param speed 目标速度（rpm）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_vmTargetSpeed(unsigned int hMaster, int nodeId, int speed, int bSDO);

/**
 * @brief 获取VM模式下的当前速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param speed 输出参数，成功时返回当前速度（rpm）
 * @param bSDO 1 使用SDO读；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_vmCurrentSpeed(unsigned int hMaster, int nodeId, int *speed, int bSDO);

/**
 * @brief 轮廓位置模式下绝对位置运动
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param target 目标位置（用户单位）
 * @param bChangeImmediatly 是否立即更新：1 立即更新；0 非立即更新
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_moveAbsolute(unsigned int hMaster, int nodeId, double position, int bChangeImmediatly, int bSDO);

/**
 * @brief 轮廓位置模式下相对位置运动
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param distance 目标位置（用户单位）
 * @param bChangeImmediatly 是否立即更新：1 立即更新；0 非立即更新
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_moveRelative(unsigned int hMaster, int nodeId, double distance, int bChangeImmediatly, int bSDO);

/**
 * @brief 设置目标位置(607A)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param fPos 目标位置（用户单位）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_targetPosition(unsigned int hMaster, int nodeId, double fPos, int bSDO);

/**
 * @brief 设置插补位置(60C1:01)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param fPos  位置（用户单位）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_ipPosition(unsigned int hMaster, int nodeId, double fPos, int bSDO);

/**
 * @brief 设置插补周期(60C2:01, 60C2:02)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param nPeriodMS  插补周期（单位：ms）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_ipPeriod(unsigned int hMaster, int nodeId, unsigned int nPeriodMS);

/**
 * @brief 获取插补周期(60C2:01, 60C2:02)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param nPeriodMS  插补周期（单位：ms）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_ipPeriod(unsigned int hMaster, int nodeId, unsigned int* nPeriodMS);

/**
 * @brief 设置目标转矩(6071)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque  目标转矩（0.001倍额定转矩）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_targetTorque(unsigned int hMaster, int nodeId, int torque, int bSDO);

/**
 * @brief 获取当前转矩(6077)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque 输出参数，成功时返回当前转矩（0.001倍额定转矩）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_currentTorque(unsigned int hMaster, int nodeId, int* torque, int bSDO);

/**
 * @brief 设置轮廓转矩模式下速度限制(2007:10h、2207:11h)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param FwrSpeedLimit  正向速度限制（单位rpm）
 * @param BwrSpeedLimit  正向速度限制（单位rpm）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_PT_SpeedLimit(unsigned int hMaster, int nodeId, unsigned short FwrSpeedLimit, unsigned short BwrSpeedLimit);

/**
 * @brief 获取轮廓转矩模式下速度限制(2007:10h、2207:11h)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param FwrSpeedLimit  正向速度限制（单位rpm）
 * @param BwrSpeedLimit  正向速度限制（单位rpm）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_PT_SpeedLimit(unsigned int hMaster, int nodeId, unsigned short* FwrSpeedLimit, unsigned short* BwrSpeedLimit);

/**
 * @brief 设置转矩斜坡
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param torqueRamp 转矩斜坡：0 无斜坡；>0 每秒钟增加的转矩值（单位：0.001倍额定转矩）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_PT_TorqueRamp(unsigned int hMaster, int nodeId, unsigned int torqueRamp);

/**
 * @brief 获取转矩斜坡
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param torqueRamp 转矩斜坡：0 无斜坡；>0 每秒钟增加的转矩值（单位：0.001倍额定转矩）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_PT_TorqueRamp(unsigned int hMaster, int nodeId, unsigned int* torqueRamp);

/**
 * @brief 快速停止当前动作
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_fastStop(unsigned int hMaster, int nodeId, int bSDO);

/**
 * @brief 清除轴故障
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_clearError(unsigned int hMaster, int nodeId, int bSDO);

/**
 * @brief 获取最新报警(603F)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param alarmCode 输出参数，执行成功时返回报警码
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_newestAlarm(unsigned int hMaster, int nodeId, unsigned int *alarmCode, int bSDO);

/**
 * @brief 获取历史报警数量(1003:00)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param alarmCode 输出参数，执行成功时返回历史报警数量
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_alarmCount(unsigned int hMaster, int nodeId, int *count);

/**
 * @brief 获取历史报警(1003:01~10h)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param index 历史报警序号,取值范围：1~16
 * @param alarmCode 输出参数，执行成功时返回报警码
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_alarm(unsigned int hMaster, int nodeId, int index, unsigned int *alarmCode);

/**
 * @brief 获取轮廓速度(6081)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param fVelocity 速度（用户单位/s）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_profileVelocity(unsigned int hMaster, int nodeId, double* fVelocity);

/**
 * @brief 获取轮廓加速度(6083)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param accel 加速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_profileAccel(unsigned int hMaster, int nodeId, double* accel);

/**
 * @brief 获取轮廓减速度(6084)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param decel 减速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_profileDecel(unsigned int hMaster, int nodeId, double* decel);

/**
 * @brief 获取快速停机减速度(6085)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param decel 减速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_quickStopDecel(unsigned int hMaster, int nodeId, double* decel);

/**
 * @brief 设置轮廓速度(6081)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param fVelocity 速度（用户单位/s）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_profileVelocity(unsigned int hMaster, int nodeId, double fVelocity);

/**
 * @brief 设置轮廓加速度(6083)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param accel 加速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_profileAccel(unsigned int hMaster, int nodeId, double accel);

/**
 * @brief 设置轮廓减速度(6084)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param decel 减速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_profileDecel(unsigned int hMaster, int nodeId, double decel);

/**
 * @brief 设置快速停机减速度(6085)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param decel 减速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_quickStopDecel(unsigned int hMaster, int nodeId, double decel);

/**
 * @brief 获取原点偏移
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param offset 原点偏移，用户单位
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_homeOffset(unsigned int hMaster, int nodeId, double* offset);

/**
 * @brief 获取原点回归速度(6099:01/02)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity1 寻找开关速度（用户单位/s）
 * @param velocity2 寻找原点速度（用户单位/s）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_goHome_velocity(unsigned int hMaster, int nodeId, double* velocity1, double* velocity2);

/**
 * @brief 获取原点回归加速度(609A)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param accel 加速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_goHome_accel(unsigned int hMaster, int nodeId, double* accel);

/**
 * @brief 设置原点偏移
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param offset 原点偏移，用户单位
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_homeOffset(unsigned int hMaster, int nodeId, double offset);

/**
 * @brief 设置原点回归速度(6099:01/02)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity1 寻找开关速度（用户单位/s）
 * @param velocity2 寻找原点速度（用户单位/s）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_goHome_velocity(unsigned int hMaster, int nodeId, double velocity1, double velocity2);

/**
 * @brief 设置原点回归加速度(609A)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param accel 加速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_goHome_accel(unsigned int hMaster, int nodeId, double accel);

/**
 * @brief 获取VM模式下的加速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param accel 输出参数，成功时返回加速度（rpm/s）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_vmAccel(unsigned int hMaster, int nodeId, double* accel);

/**
 * @brief 设置VM模式下的加速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param deltaV 速度变化量（rpm）
 * @param deltaT 时间变化量（S）
 *        加速度 = deltaV/deltaT
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_vmAccel(unsigned int hMaster, int nodeId, unsigned int deltaV, unsigned int deltaT);

/**
 * @brief 获取VM模式下的减速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param accel 输出参数，成功时返回减速度（rpm/s）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_vmDecel(unsigned int hMaster, int nodeId, double* decel);

/**
 * @brief 设置VM模式下的减速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param deltaV 速度变化量（rpm）
 * @param deltaT 时间变化量（S）
 *        减速度 = deltaV/deltaT
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_vmDecel(unsigned int hMaster, int nodeId, unsigned int deltaV, unsigned int deltaT);

/**
 * @brief 通过6069获取当前速度
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity 输出参数，执行成功时返回当前速度（用户单位/s）
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_currentVelocity(unsigned int hMaster, int nodeId, double* velocity, int bSDO);

/**
 * @brief 通过606C获取当前速度
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity 输出参数，执行成功时返回当前速度（用户单位/s）
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_currentVelocity2(unsigned int hMaster, int nodeId, double* velocity, int bSDO);

/**
 * @brief 通过606C获取当前电机速度
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param speed 输出参数，执行成功时返回当前速度（rpm）
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_currentMotorSpeed(unsigned int hMaster, int nodeId, int* speed, int bSDO);

/**
 * @brief 通过6064获取当前位置
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param position 输出参数，执行成功时返回当前位置
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_currentPosition(unsigned int hMaster, int nodeId, double* position, int bSDO);

/**
 * @brief 获取位置限制值(607D:01/02)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param minPos 输出参数，执行成功时返回最小极限位置
 * @param maxPos 输出参数，执行成功时返回最大极限位置
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_posLimit(unsigned int hMaster, int nodeId, double* minPos, double* maxPos);

/**
 * @brief 设置位置限制值(607D:01/02)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param pos 输出参数，执行成功时返回最大极限位置
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_posLimit(unsigned int hMaster, int nodeId, double minPos, double maxPos);

/**
 * @brief 获取最大速度(607F)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity 输出参数，成功时返回最大速度（用户单位/s）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_maxVelocity(unsigned int hMaster, int nodeId, double* velocity);


/**
 * @brief 设置最大速度(607F)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity 最大速度（用户单位/s）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_maxVelocity(unsigned int hMaster, int nodeId, double velocity);


/**
 * @brief 获取最大电机速度(6080)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque 输出参数，成功时返回最大电机速度（rpm）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_maxMotorSpeed(unsigned int hMaster, int nodeId, unsigned int* speed);

/**
 * @brief 设置最大电机速度(6080)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque  最大电机速度（rpm）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_maxMotorSpeed(unsigned int hMaster, int nodeId, unsigned int speed);

/**
 * @brief 获取最大转矩(6072)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque 输出参数，成功时返回最大转矩（0.001倍额定转矩）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_maxTorque(unsigned int hMaster, int nodeId, unsigned int* torque);

/**
 * @brief 设置最大转矩(6072)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque  最大转矩（0.001倍额定转矩）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_maxTorque(unsigned int hMaster, int nodeId, unsigned int torque);

/**
 * @brief 获取VM模式下的速度限制值
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param minSpeed 最小速度（rpm）
 * @param maxSpeed 最大速度（rpm）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_vmSpeedLimit(unsigned int hMaster, int nodeId, unsigned int* minSpeed, unsigned int* maxSpeed);

/**
 * @brief 设置VM模式下的速度限制值
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param minSpeed 最小速度（rpm）
 * @param maxSpeed 最大速度（rpm）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_vmSpeedLimit(unsigned int hMaster, int nodeId, unsigned int minSpeed, unsigned int maxSpeed);

/**
 * @brief 设置用户单位的转换系数
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param factor 转换系数（电机编码器单位/用户单位）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_unitsFactor(unsigned int hMaster, int nodeId, double factor);

/**
 * @brief 获取用户单位的转换系数
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param factor 转换系数（电机编码器单位/用户单位）
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_unitsFactor(unsigned int hMaster, int nodeId, double* factor);

/**
 * @brief  设置DO输出
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param nDOs DO值：每位表示一路DO：bit0表示DO1；bit1表示DO2，以此类推
 *                  1 输出高电平；0 输出低电平
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_DOs(unsigned int hMaster, int nodeId, unsigned int nDOs, int bSDO);


/**
 * @brief  设置VDI值
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param nVDIs VDI值：每位表示一路VDI：bit0表示VDI1；bit1表示VDI2，以此类推
 *                  1 输出高电平；0 输出低电平
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_set_VDIs(unsigned int hMaster, int nodeId, unsigned int nVDIs);

/**
 * @brief  读取DI输入
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param nDIs DI值：每位表示一路DI：bit0表示DI1；bit1表示DI2，以此类推
 *                  1 输入高电平；0 输入低电平
 * @param bSDO 1 使用SDO读取；0 使用PDO
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_get_DIs(unsigned int hMaster, int nodeId, unsigned int* nDIs, int bSDO);

/**
 * @brief 保存所有参数到设备
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param timeoutMS 超时时间，单位：ms
 * @return 0 成功；其它 失败
 */
NIMSERVOSDKSHARED_EXPORT
int NIMAPI Nim_save_AllParams(unsigned int hMaster, int nodeId, int timeoutMS);

#ifdef __cplusplus
}
#endif


#endif // NIMSERVOSDK_H
