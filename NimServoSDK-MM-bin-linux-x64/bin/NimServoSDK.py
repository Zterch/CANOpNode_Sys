#!/usr/bin/env python
# -*- coding: utf-8 -*-

import ctypes, os, types, sys, time
#from enum import Enum

if os.name in ('nt', 'ce'):
    from _ctypes import LoadLibrary as dlopen
    from _ctypes import FreeLibrary as dlclose
elif os.name == 'posix':
    from _ctypes import dlopen,dlclose

SDKHandle = None 
_SDKlibraryHandle = None

class CanBaudRate():
    CAN_BT_10K = 0
    CAN_BT_20K = 1
    CAN_BT_50K = 2
    CAN_BT_100K = 3
    CAN_BT_125K = 4
    CAN_BT_250K = 5
    CAN_BT_500K = 6
    CAN_BT_800K = 7
    CAN_BT_1000K = 8
    
class ServoWorkMode():
    SERVO_PP_MODE = 1  #轮廓位置模式     PP
    SERVO_VM_MODE = 2  #速度模式        VM
    SERVO_PV_MODE = 3  #轮廓速度模式     PV
    SERVO_PT_MODE = 4  #轮廓转矩模式     PT
    SERVO_HM_MODE = 6  #原点回归模式     HM
    SERVO_IP_MODE = 7  #位置插补模式     IP
    SERVO_CSP_MODE = 8  #循环同步位置模式  CSP
    SERVO_CSV_MODE = 9  #循环同步速度模式  CSV
    SERVO_CST_MODE = 10 #循环同步转矩模式  CST
        
class ServoSDK_Error():
    ServoSDK_NoError = 0           #没有错误
    ServoSDK_NotRegisted = 1           #SDK没有注册
    ServoSDK_NotInitialized = 2        #SDK没有初始化
    ServoSDK_UnsupportedCommType = 3   #不支持的通信方式
    ServoSDK_ParamError = 4            #输入参数错误
    ServoSDK_CreateMasterFailed = 5    #创建主站失败
    ServoSDK_MasterNotExist = 6        #主站不存在
    ServoSDK_MasterStartFailed = 7     #主站启动失败
    ServoSDK_MasterNotRunning = 8      #主站未运行
    ServoSDK_SlaveNotOnline = 9        #从站不在线
    ServoSDK_LoadParamSheetFailed = 10  #加载参数表错误
    ServoSDK_ParamNotExist = 11         #请求的参数不存在
    ServoSDK_ReadSDOFailed = 12         #读SDO失败
    ServoSDK_WriteSDOFailed = 13        #写SDO失败
    ServoSDK_OperationNotAllowed = 14   #操作不允许
    ServoSDK_MasterInternalError = 15   #主站内部错误
    ServoSDK_SlaveInternalError = 16    #从站内部错误
    ServoSDK_Cia402ModeError = 17       #从站402模式错误
    ServoSDK_ReadWorkModeFailed = 18    #读取工作模式失败
    ServoSDK_ReadStatusWordFailed = 19  #读取状态字失败
    ServoSDK_ReadCurrentPosFailed = 20  #读取当前位置失败
    ServoSDK_ReadRPDOConfigFailed = 21  #读取PDO配置失败
    ServoSDK_ReadTPDOConfigFailed = 22  #读取PDO配置失败
    ServoSDK_WriteControlWordFailed = 23  #写控制字失败
    ServoSDK_WriteTargetPosFailed = 24  #写目标位置失败
    ServoSDK_WriteTargetVelFailed = 25  #写目标速度失败
    ServoSDK_WriteGoHomeTypeFailed = 26  #写原点回归方式失败
    ServoSDK_GetHostInfoFailed = 27     #获取主机信息失败
    ServoSDK_SaveParamsFailed = 28      #保存参数失败
    ServoSDK_NoAvailableDevice = 29     #没有可用的设备
    ServoSDK_Unknown = 255          #未知错误

'''
 * @brief SDK初始化
 * @param strSdkPath SDK库加载路径
 * @return 0 成功；其它 失败
 '''


def Nim_init(strSdkPath):
    global SDKHandle
    global _SDKlibraryHandle
    if SDKHandle is None:
        if os.name in ('nt', 'ce'):
            _SDKlibraryHandle = ctypes.WinDLL(os.path.join(strSdkPath, 'NimServoSDK.dll'))
            SDKHandle = ctypes.CDLL(None, handle=_SDKlibraryHandle._handle)
        elif os.name == 'posix':
            _SDKlibraryHandle = ctypes.CDLL(os.path.join(strSdkPath, 'libNimServoSDK.so'))
            SDKHandle = ctypes.CDLL(None, handle=_SDKlibraryHandle._handle)

    _Nim_init = SDKHandle.Nim_init
    _Nim_init.restype = ctypes.c_int
    _Nim_init.argtypes = [ctypes.c_char_p]

    return _Nim_init(strSdkPath.encode('utf-8'))


'''
 * @brief SDK 反初始化
'''
def Nim_clean():
    global SDKHandle
    global _SDKlibraryHandle
    _Nim_clean = SDKHandle.Nim_clean
    _Nim_clean.restype = None

    _Nim_clean()

    SDKHandle = None
    if _SDKlibraryHandle is not None:
        dlclose(int(_SDKlibraryHandle._handle))
    _SDKlibraryHandle = None

    
'''
 * @brief 设置日志输出标志
 * @param bit0=1输出到控制台；bit1=1输出到文件
'''
def Nim_setLogFlags(nFlags):
    global SDKHandle
    global _SDKlibraryHandle
    if SDKHandle is None:
        if os.name in ('nt', 'ce'):
            _SDKlibraryHandle = ctypes.WinDLL(os.path.join("", 'NimServoSDK.dll'))
            SDKHandle = ctypes.CDLL(None, handle=_SDKlibraryHandle._handle)
        elif os.name == 'posix':
            _SDKlibraryHandle = ctypes.CDLL(os.path.join("", 'libNimServoSDK.so'))
            SDKHandle = ctypes.CDLL(None, handle=_SDKlibraryHandle._handle)
    _Nim_setLogFlags = SDKHandle.Nim_setLogFlags
    _Nim_setLogFlags.restype = ctypes.c_int
    _Nim_setLogFlags.argtypes = [ctypes.c_int]
    
    _Nim_setLogFlags(nFlags)
    
'''
 * @brief 获取日志输出标志
 * @return bit0=1输出到控制台；bit1=1输出到文件
'''  
def Nim_getLogFlags():
    _Nim_getLogFlags = SDKHandle.Nim_getLogFlags
    _Nim_getLogFlags.restype = ctypes.c_int
    _Nim_getLogFlags.argtypes = None
    
    return _Nim_getLogFlags()

'''
 * @brief 创建主站对象
 * @param nCommType 通信方式：0 CANopen；1 EtherCAT; 2 Modbus
 * @return [nRes, hMaster]
 *          nRes 执行结果：0 成功；其它 失败
 *          hMaster 执行成功时返回主站句柄
'''
def Nim_create_master(nCommType):
    _Nim_create_master = SDKHandle.Nim_create_master
    _Nim_create_master.restype = ctypes.c_int
    _Nim_create_master.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
    hMaster = ctypes.c_uint32()
    nRes = _Nim_create_master(nCommType, ctypes.byref(hMaster))
    
    return [nRes, hMaster.value]
    
'''
 * @brief 销毁主站对象
 * @param handle 由Nim_create_master函数创建的主站对象句柄
 * @return 0 成功；其它 失败
'''
def Nim_destroy_master(hMaster):
    _Nim_destroy_master = SDKHandle.Nim_destroy_master
    _Nim_destroy_master.restype = ctypes.c_int
    _Nim_destroy_master.argtypes = [ctypes.c_uint32]

    return _Nim_destroy_master(hMaster)

'''
 * @brief 启动通信主站
 * @param hMaster 主站对象句柄
 * @param conn_str 连接字符串，json格式，具体内容参见使用手册
 * @return 0 成功；其它 失败
'''
def Nim_master_run(hMaster, conn_str):
    _Nim_master_run = SDKHandle.Nim_master_run
    _Nim_master_run.restype = ctypes.c_int
    _Nim_master_run.argtypes = [ctypes.c_uint32, ctypes.c_char_p]

    return _Nim_master_run(hMaster, conn_str.encode('utf-8'))

'''
 * @brief 关闭通信主站
 * @param hMaster 主站对象句柄
 * @return 0 成功；其它 失败
'''
def Nim_master_stop(hMaster):
    _Nim_master_stop = SDKHandle.Nim_master_stop
    _Nim_master_stop.restype = ctypes.c_int
    _Nim_master_stop.argtypes = [ctypes.c_uint32]

    return _Nim_master_stop(hMaster)

'''
 * @brief 主站进入PreOP模式
 * @param hMaster 主站对象句柄
 * @return 0 成功；其它 失败
'''
def Nim_master_changeToPreOP(hMaster):
    _Nim_master_changeToPreOP = SDKHandle.Nim_master_changeToPreOP
    _Nim_master_changeToPreOP.restype = ctypes.c_int
    _Nim_master_changeToPreOP.argtypes = [ctypes.c_uint32]

    return _Nim_master_changeToPreOP(hMaster)

'''
 * @brief 主站进入OP模式
 * @param hMaster 主站对象句柄
 * @return 0 成功；其它 失败
'''
def Nim_master_changeToOP(hMaster):
    _Nim_master_changeToOP = SDKHandle.Nim_master_changeToOP
    _Nim_master_changeToOP.restype = ctypes.c_int
    _Nim_master_changeToOP.argtypes = [ctypes.c_uint32]

    return _Nim_master_changeToOP(hMaster)

'''
 * @brief 按照指定的地址范围扫描从站是否在线
 * @param hMaster 主站对象句柄
 * @param from 起始地址
 * @param to 结束地址
 * @return 0 成功；其它 失败
'''
def Nim_scan_nodes(hMaster, fromAddr, toAddr):
    _Nim_scan_nodes = SDKHandle.Nim_scan_nodes
    _Nim_scan_nodes.restype = ctypes.c_int
    _Nim_scan_nodes.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int]

    return _Nim_scan_nodes(hMaster, fromAddr, toAddr)

'''
 * @brief 查询从站是否在线
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @return 非零 在线；0 不在线
 '''
def Nim_is_online(hMaster, nodeId):
    _Nim_is_online = SDKHandle.Nim_is_online
    _Nim_is_online.restype = ctypes.c_int
    _Nim_is_online.argtypes = [ctypes.c_uint32, ctypes.c_int]

    return _Nim_is_online(hMaster, nodeId)

'''
 * @brief 读取从站PDO配置
 * @param hMaster 主站对象句柄
 * @return 0 成功；其它 失败
 '''
def Nim_read_PDOConfig(hMaster, nodeId):
    _Nim_read_PDOConfig = SDKHandle.Nim_read_PDOConfig
    _Nim_read_PDOConfig.restype = ctypes.c_int
    _Nim_read_PDOConfig.argtypes = [ctypes.c_uint32, ctypes.c_int]

    return _Nim_read_PDOConfig(hMaster, nodeId)

'''
 * @brief 加载电机参数表
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param db_name 参数表数据库文件名
 * @return 0 成功；其它 失败
 '''
def Nim_load_params(hMaster, nodeId, db_name):
    _Nim_load_params = SDKHandle.Nim_load_params
    _Nim_load_params.restype = ctypes.c_int
    _Nim_load_params.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_char_p]

    return _Nim_load_params(hMaster, nodeId, db_name.encode('utf-8'))

'''
 * @brief 读取从站参数
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param pParamNO 参数编号
 * @param bSDO 1 使用SDO读；0 使用PDO
 * @return [nRes, uiValue]
 *          nRes 执行结果：0 成功；其它 失败
 *          uiValue 执行成功时返回参数值
 '''
def Nim_get_param_value(hMaster, nodeId, strParamNO, bSDO):
    _Nim_get_param_value = SDKHandle.Nim_get_param_value
    _Nim_get_param_value.restype = ctypes.c_int
    _Nim_get_param_value.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint32), ctypes.c_int]

    uiValue = ctypes.c_uint32()
    nRes = _Nim_get_param_value(hMaster, nodeId, strParamNO, ctypes.byref(uiValue), bSDO)
    return [nRes, uiValue.value]

'''
 * @brief 设置从站参数
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param pParamNO 参数编号
 * @param uiValue 参数值
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_param_value(hMaster, nodeId, strParamNO, nValue, bSDO):
    _Nim_set_param_value = SDKHandle.Nim_set_param_value
    _Nim_set_param_value.restype = ctypes.c_int
    _Nim_set_param_value.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_int]

    return _Nim_set_param_value(hMaster, nodeId, strParamNO, nValue, bSDO)

'''
 * @brief 电机抱机
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_power_on(hMaster, nodeId, bSDO):
    _Nim_power_on = SDKHandle.Nim_power_on
    _Nim_power_on.restype = ctypes.c_int
    _Nim_power_on.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int]

    return _Nim_power_on(hMaster, nodeId, bSDO)

'''
 * @brief 电机脱机
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return true 成功；false 失败
 '''
def Nim_power_off(hMaster, nodeId, bSDO):
    _Nim_power_off = SDKHandle.Nim_power_off
    _Nim_power_off.restype = ctypes.c_int
    _Nim_power_off.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int]

    return _Nim_power_off(hMaster, nodeId, bSDO)

'''
 * @brief 设置控制字(6040)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param cw 控制字
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_controlWord(hMaster, nodeId, cw, bSDO):
    _Nim_set_controlWord = SDKHandle.Nim_set_controlWord
    _Nim_set_controlWord.restype = ctypes.c_int
    _Nim_set_controlWord.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint16, ctypes.c_int]

    return _Nim_set_controlWord(hMaster, nodeId, cw, bSDO)

'''
 * @brief 获取电机状态字(6041)
 * @param hMaster 主站对象句柄
 * @param nodeId 电机地址
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return [nRes, status_word]
 *          nRes 执行结果：0 成功；其它 失败
 *          status_word 执行成功时返回状态字
 '''
def Nim_get_statusWord(hMaster, nodeId, bSDO):
    _Nim_get_statusWord = SDKHandle.Nim_get_statusWord
    _Nim_get_statusWord.restype = ctypes.c_int
    _Nim_get_statusWord.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint16), ctypes.c_int]

    uiValue = ctypes.c_uint16()
    nRes = _Nim_get_statusWord(hMaster, nodeId, ctypes.byref(uiValue), bSDO)
    return [nRes, uiValue.value]

'''
 * @brief 设置电机工作模式（6060,在脱机状态下设置）
 * @param hMaster 主站对象句柄
 * @param nodeId 电机地址
 * @param mode 模式
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_workMode(hMaster, nodeId, mode, bSDO):
    _Nim_set_workMode = SDKHandle.Nim_set_workMode
    _Nim_set_workMode.restype = ctypes.c_int
    _Nim_set_workMode.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int, ctypes.c_int]

    return _Nim_set_workMode(hMaster, nodeId, mode, bSDO)

'''
 * @brief 获取电机工作模式显示值(6061)
 * @param hMaster 主站对象句柄
 * @param nodeId 电机地址
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return  [nRes, mode_display]
 *          nRes 执行结果：0 成功；其它 失败
 *          mode_display 执行成功时返回模式显示值
 '''
def Nim_get_workModeDisplay(hMaster, nodeId, bSDO):
    _Nim_get_workModeDisplay = SDKHandle.Nim_get_workModeDisplay
    _Nim_get_workModeDisplay.restype = ctypes.c_int
    _Nim_get_workModeDisplay.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int]

    iValue = ctypes.c_int()
    nRes = _Nim_get_workModeDisplay(hMaster, nodeId, ctypes.byref(iValue), bSDO)
    return [nRes, iValue.value]


'''
 * @brief 设置原点回归方式
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param type 回原点方式
 * @return 0 成功；其它 失败
'''
def Nim_set_homeType(hMaster, nodeId, type):
    _Nim_set_homeType = SDKHandle.Nim_set_homeType
    _Nim_set_homeType.restype = ctypes.c_int
    _Nim_set_homeType.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_ubyte]
    
    return _Nim_set_homeType(hMaster, nodeId, type)


'''
 * @brief 获取原点回归方式
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param type 回原点方式
 * @return 0 成功；其它 失败
'''
def Nim_get_homeType(hMaster, nodeId):
    _Nim_get_homeType = SDKHandle.Nim_get_homeType
    _Nim_get_homeType.restype = ctypes.c_int
    _Nim_get_homeType.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_ubyte)]

    type = ctypes.c_ubyte()
    result = _Nim_get_homeType(hMaster, nodeId, ctypes.byref(type))
    return [result, type.value]

'''
 * @brief 原点回归
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_goHome(hMaster, nodeId, bSDO):
    _Nim_goHome = SDKHandle.Nim_goHome
    _Nim_goHome.restype = ctypes.c_int
    _Nim_goHome.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int]

    return _Nim_goHome(hMaster, nodeId, bSDO)

'''
 * @brief 轮廓速度模式下正转
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param fVelocity 速度（用户单位/s）
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_forward(hMaster, nodeId, fVelocity, bSDO):
    _Nim_forward = SDKHandle.Nim_forward
    _Nim_forward.restype = ctypes.c_int
    _Nim_forward.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_int]

    return _Nim_forward(hMaster, nodeId, fVelocity, bSDO)

'''
 * @brief 轮廓速度模式下反转
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param fVelocity 速度（用户单位/s）
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_backward(hMaster, nodeId, fVelocity, bSDO):
    _Nim_backward = SDKHandle.Nim_backward
    _Nim_backward.restype = ctypes.c_int
    _Nim_backward.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_int]

    return _Nim_backward(hMaster, nodeId, fVelocity, bSDO)

'''
 * @brief 设置目标速度(60FF)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param fVelocity 目标速度（用户单位/s）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_targetVelocity(hMaster, nodeId, fVelocity, bSDO):
    _Nim_set_targetVelocity = SDKHandle.Nim_set_targetVelocity
    _Nim_set_targetVelocity.restype = ctypes.c_int
    _Nim_set_targetVelocity.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_int]

    return _Nim_set_targetVelocity(hMaster, nodeId, fVelocity, bSDO)
    
'''
 * @brief 设置VM模式下的目标速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param speed 目标速度（rpm）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_vmTargetSpeed(hMaster, nodeId, nSpeed, bSDO):
    _Nim_set_vmTargetSpeed = SDKHandle.Nim_set_vmTargetSpeed
    _Nim_set_vmTargetSpeed.restype = ctypes.c_int
    _Nim_set_vmTargetSpeed.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int, ctypes.c_int]

    return _Nim_set_vmTargetSpeed(hMaster, nodeId, nSpeed, bSDO)

'''
 * @brief 获取VM模式下的当前速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param bSDO 1 使用SDO读；0 使用PDO
 * @return [nRes, speed]
 *          nRes 执行结果：0 成功；其它 失败
 *          speed 执行成功时返回当前速度（rpm）
 '''
def Nim_get_vmCurrentSpeed(hMaster, nodeId, bSDO):
    _Nim_get_vmCurrentSpeed = SDKHandle.Nim_get_vmCurrentSpeed
    _Nim_get_vmCurrentSpeed.restype = ctypes.c_int
    _Nim_get_vmCurrentSpeed.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int]

    nSpeed = ctypes.c_int()
    nRes = _Nim_get_vmCurrentSpeed(hMaster, nodeId, ctypes.byref(nSpeed), bSDO)
    return [nRes, nSpeed.value]

'''
 * @brief 轮廓位置模式下绝对位置运动
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param target 目标位置（用户单位）
 * @param bChangeImmediatly 是否立即更新：1 立即更新；0 非立即更新
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_moveAbsolute(hMaster, nodeId, position, bChangeImmediatly, bSDO):
    _Nim_moveAbsolute = SDKHandle.Nim_moveAbsolute
    _Nim_moveAbsolute.restype = ctypes.c_int
    _Nim_moveAbsolute.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_int]

    return _Nim_moveAbsolute(hMaster, nodeId, position, bChangeImmediatly, bSDO)

'''
 * @brief 轮廓位置模式下相对位置运动
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param distance 目标位置（用户单位）
 * @param bChangeImmediatly 是否立即更新：1 立即更新；0 非立即更新
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_moveRelative(hMaster, nodeId, distance, bChangeImmediatly, bSDO):
    _Nim_moveRelative = SDKHandle.Nim_moveRelative
    _Nim_moveRelative.restype = ctypes.c_int
    _Nim_moveRelative.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_int]

    return _Nim_moveRelative(hMaster, nodeId, distance, bChangeImmediatly, bSDO)

'''
 * @brief 设置目标位置(607A)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param fPos 目标位置（用户单位）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_targetPosition(hMaster, nodeId, position, bSDO):
    _Nim_set_targetPosition = SDKHandle.Nim_set_targetPosition
    _Nim_set_targetPosition.restype = ctypes.c_int
    _Nim_set_targetPosition.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_int]

    return _Nim_set_targetPosition(hMaster, nodeId, position, bSDO)

'''
 * @brief 设置插补位置(60C1:01)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param fPos  位置（用户单位）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_ipPosition(hMaster, nodeId, position, bSDO):
    _Nim_set_ipPosition = SDKHandle.Nim_set_ipPosition
    _Nim_set_ipPosition.restype = ctypes.c_int
    _Nim_set_ipPosition.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_int]

    return _Nim_set_ipPosition(hMaster, nodeId, position, bSDO)
 
'''
 * @brief 设置插补位置(60C2:01, 60C2:02)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param nPeriodMS  插补周期（单位：ms）
 * @return 0 成功；其它 失败
'''
def Nim_set_ipPeriod(hMaster, nodeId, nPeriodMS):
    _Nim_set_ipPeriod = SDKHandle.Nim_set_ipPeriod
    _Nim_set_ipPeriod.restype = ctypes.c_int
    _Nim_set_ipPeriod.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32]
    
    return _Nim_set_ipPeriod(hMaster, nodeId, nPeriodMS)
    
'''
 * @brief 获取插补位置(60C2:01, 60C2:02)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param nPeriodMS  插补周期（单位：ms）
 * @return 0 成功；其它 失败
'''
def Nim_get_ipPeriod(hMaster, nodeId):
    _Nim_get_ipPeriod = SDKHandle.Nim_get_ipPeriod
    _Nim_get_ipPeriod.restype = ctypes.c_int
    _Nim_get_ipPeriod.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]

    nPeriodMS = ctypes.c_uint32()
    nRes = _Nim_get_ipPeriod(hMaster, nodeId, ctypes.byref(nPeriodMS))
    return [nRes, nPeriodMS.value]
    
'''
 * @brief 设置目标转矩(6071)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque  目标转矩（0.001倍额定转矩）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_targetTorque(hMaster, nodeId, torque, bSDO):
    _Nim_set_targetTorque = SDKHandle.Nim_set_targetTorque
    _Nim_set_targetTorque.restype = ctypes.c_int
    _Nim_set_targetTorque.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int, ctypes.c_int]

    return _Nim_set_targetTorque(hMaster, nodeId, torque, bSDO)
    
'''
 * @brief 获取当前转矩(6077)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque 输出参数，成功时返回当前转矩（0.001倍额定转矩）
 * @param bSDO 1 使用SDO写；0 使用PDO
 * @return [nRes, Torque]
 *          nRes 执行结果：0 成功；其它 失败
 *          Torque 执行成功时返回当前转矩（0.001倍额定转矩）
 '''
def Nim_get_currentTorque(hMaster, nodeId, bSDO):
    _Nim_get_currentTorque = SDKHandle.Nim_get_currentTorque
    _Nim_get_currentTorque.restype = ctypes.c_int
    _Nim_get_currentTorque.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int]

    nTorque = ctypes.c_int()
    nRes = _Nim_get_currentTorque(hMaster, nodeId, ctypes.byref(nTorque), bSDO)
    return [nRes, nTorque.value]


'''
 * @brief 设置轮廓转矩模式下速度限制(2007:10h、2207:11h)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param FwrSpeedLimit  正向速度限制（单位rpm）
 * @param BwrSpeedLimit  正向速度限制（单位rpm）
 * @return 0 成功；其它 失败
'''
def Nim_set_PT_SpeedLimit(hMaster, nodeId, FwrSpeedLimit, BwrSpeedLimit):
    _Nim_set_PT_SpeedLimit = SDKHandle.Nim_set_PT_SpeedLimit
    _Nim_set_PT_SpeedLimit.restype = ctypes.c_int
    _Nim_set_PT_SpeedLimit.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_ushort, ctypes.c_ushort]

    return _Nim_set_PT_SpeedLimit(hMaster, nodeId, FwrSpeedLimit, BwrSpeedLimit)
    
'''
 * @brief 获取轮廓转矩模式下速度限制(2007:10h、2207:11h)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param FwrSpeedLimit  正向速度限制（单位rpm）
 * @param BwrSpeedLimit  正向速度限制（单位rpm）
 * @return 0 成功；其它 失败
'''
def Nim_get_PT_SpeedLimit(hMaster, nodeId):
    _Nim_get_PT_SpeedLimit = SDKHandle.Nim_get_PT_SpeedLimit
    _Nim_get_PT_SpeedLimit.restype = ctypes.c_int
    _Nim_get_PT_SpeedLimit.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_ushort), ctypes.POINTER(ctypes.c_ushort)]

    FwrSpeedLimit = ctypes.c_ushort()
    BwrSpeedLimit = ctypes.c_ushort()
    result = _Nim_get_PT_SpeedLimit(hMaster, nodeId, ctypes.byref(FwrSpeedLimit), ctypes.byref(BwrSpeedLimit))
    
    return [result, FwrSpeedLimit.value, BwrSpeedLimit.value]
    
    
'''
 * @brief 设置转矩斜坡
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param torqueRamp 转矩斜坡：0 无斜坡；>0 每秒钟增加的转矩值（单位：0.001倍额定转矩）
 * @return 0 成功；其它 失败
'''
def Nim_set_PT_TorqueRamp(hMaster, nodeId, torqueRamp):
    _Nim_set_PT_TorqueRamp = SDKHandle.Nim_set_PT_TorqueRamp
    _Nim_set_PT_TorqueRamp.restype = ctypes.c_int
    _Nim_set_PT_TorqueRamp.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint]

    return _Nim_set_PT_TorqueRamp(hMaster, nodeId, torqueRamp)
'''
 * @brief 获取转矩斜坡
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param torqueRamp 转矩斜坡：0 无斜坡；>0 每秒钟增加的转矩值（单位：0.001倍额定转矩）
 * @return 0 成功；其它 失败
'''
def Nim_get_PT_TorqueRamp(hMaster, nodeId):
    _Nim_get_PT_TorqueRamp = SDKHandle.Nim_get_PT_TorqueRamp
    _Nim_get_PT_TorqueRamp.restype = ctypes.c_int
    _Nim_get_PT_TorqueRamp.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint)]

    torqueRamp = ctypes.c_uint()
    result = _Nim_get_PT_TorqueRamp(hMaster, nodeId, ctypes.byref(torqueRamp))
    
    return [result, torqueRamp.value]

'''
 * @brief 快速停止当前动作
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_fastStop(hMaster, nodeId, bSDO):
    _Nim_fastStop = SDKHandle.Nim_fastStop
    _Nim_fastStop.restype = ctypes.c_int
    _Nim_fastStop.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int]

    return _Nim_fastStop(hMaster, nodeId, bSDO)

'''
 * @brief 清除轴故障
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_clearError(hMaster, nodeId, bSDO):
    _Nim_clearError = SDKHandle.Nim_clearError
    _Nim_clearError.restype = ctypes.c_int
    _Nim_clearError.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int]

    return _Nim_clearError(hMaster, nodeId, bSDO)

'''
 * @brief 获取最新报警(603F)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param alarmCode 输出参数，执行成功时返回报警码
 * @param bSDO 1 使用SDO控制；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_get_newestAlarm(hMaster, nodeId, bSDO):
    _Nim_get_newestAlarm = SDKHandle.Nim_get_newestAlarm
    _Nim_get_newestAlarm.restype = ctypes.c_int
    _Nim_get_newestAlarm.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32), ctypes.c_int]

    alarmCode = ctypes.c_uint32()
    nRes = _Nim_get_newestAlarm(hMaster, nodeId, ctypes.byref(alarmCode), bSDO)
    return [nRes, alarmCode.value]

'''
 * @brief 获取历史报警数量(1003:00)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param alarmCode 输出参数，执行成功时返回历史报警数量
 * @return 0 成功；其它 失败
 '''
def Nim_get_alarmCount(hMaster, nodeId):
    _Nim_get_alarmCount = SDKHandle.Nim_get_alarmCount
    _Nim_get_alarmCount.restype = ctypes.c_int
    _Nim_get_alarmCount.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]

    count = ctypes.c_int()
    nRes = _Nim_get_alarmCount(hMaster, nodeId, ctypes.byref(count))
    return [nRes, count.value]

'''
 * @brief 获取历史报警(1003:01~10h)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param index 历史报警序号,取值范围：1~16
 * @param alarmCode 输出参数，执行成功时返回报警码
 * @return 0 成功；其它 失败
 '''
def Nim_get_alarm(hMaster, nodeId, index):
    _Nim_get_alarm = SDKHandle.Nim_get_alarm
    _Nim_get_alarm.restype = ctypes.c_int
    _Nim_get_alarm.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int , ctypes.POINTER(ctypes.c_uint32)]

    alarmCode = ctypes.c_uint32()
    nRes = _Nim_get_alarm(hMaster, nodeId, index, ctypes.byref(alarmCode))
    return [nRes, alarmCode.value]

'''
 * @brief 获取轮廓速度(6081)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param fVelocity 速度（用户单位/s）
 * @return 0 成功；其它 失败
 '''
def Nim_get_profileVelocity(hMaster, nodeId):
    _Nim_get_profileVelocity = SDKHandle.Nim_get_profileVelocity
    _Nim_get_profileVelocity.restype = ctypes.c_int
    _Nim_get_profileVelocity.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    fVelocity = ctypes.c_double()
    nRes = _Nim_get_profileVelocity(hMaster, nodeId, ctypes.byref(fVelocity))
    return [nRes, fVelocity.value]

'''
 * @brief 获取轮廓加速度(6083)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param accel 加速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 '''
def Nim_get_profileAccel(hMaster, nodeId):
    _Nim_get_profileAccel = SDKHandle.Nim_get_profileAccel
    _Nim_get_profileAccel.restype = ctypes.c_int
    _Nim_get_profileAccel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    accel = ctypes.c_double()
    nRes = _Nim_get_profileAccel(hMaster, nodeId, ctypes.byref(accel))
    return [nRes, accel.value]
    
'''
 * @brief 获取轮廓减速度(6084)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param decel 减速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 '''
def Nim_get_profileDecel(hMaster, nodeId):
    _Nim_get_profileDecel = SDKHandle.Nim_get_profileDecel
    _Nim_get_profileDecel.restype = ctypes.c_int
    _Nim_get_profileDecel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    decel = ctypes.c_double()
    nRes = _Nim_get_profileDecel(hMaster, nodeId, ctypes.byref(decel))
    return [nRes, decel.value]

'''
 * @brief 获取快速停机减速度(6085)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param decel 减速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 '''
def Nim_get_quickStopDecel(hMaster, nodeId):
    _Nim_get_quickStopDecel = SDKHandle.Nim_get_quickStopDecel
    _Nim_get_quickStopDecel.restype = ctypes.c_int
    _Nim_get_quickStopDecel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    decel = ctypes.c_double()
    nRes = _Nim_get_quickStopDecel(hMaster, nodeId, ctypes.byref(decel))
    return [nRes, decel.value]

'''
 * @brief 设置轮廓速度(6081)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param fVelocity 速度（用户单位/s）
 * @return 0 成功；其它 失败
 '''
def Nim_set_profileVelocity(hMaster, nodeId, velocity):
    _Nim_set_profileVelocity = SDKHandle.Nim_set_profileVelocity
    _Nim_set_profileVelocity.restype = ctypes.c_int
    _Nim_set_profileVelocity.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double]

    return _Nim_set_profileVelocity(hMaster, nodeId, velocity)

'''
 * @brief 设置轮廓加速度(6083)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param accel 加速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 '''
def Nim_set_profileAccel(hMaster, nodeId, accel):
    _Nim_set_profileAccel = SDKHandle.Nim_set_profileAccel
    _Nim_set_profileAccel.restype = ctypes.c_int
    _Nim_set_profileAccel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double]

    return _Nim_set_profileAccel(hMaster, nodeId, accel)

'''
 * @brief 设置轮廓减速度(6084)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param decel 减速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 '''
def Nim_set_profileDecel(hMaster, nodeId, decel):
    _Nim_set_profileDecel = SDKHandle.Nim_set_profileDecel
    _Nim_set_profileDecel.restype = ctypes.c_int
    _Nim_set_profileDecel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double]

    return _Nim_set_profileDecel(hMaster, nodeId, decel)

'''
 * @brief 设置快速停机减速度(6085)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param decel 减速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 '''
def Nim_set_quickStopDecel(hMaster, nodeId, decel):
    _Nim_set_quickStopDecel = SDKHandle.Nim_set_quickStopDecel
    _Nim_set_quickStopDecel.restype = ctypes.c_int
    _Nim_set_quickStopDecel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double]

    return _Nim_set_quickStopDecel(hMaster, nodeId, decel)

'''
 * @brief 获取原点偏移
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param offset 原点偏移，用户单位
 * @return 0 成功；其它 失败
 '''
def Nim_get_homeOffset(hMaster, nodeId):
    _Nim_get_homeOffset = SDKHandle.Nim_get_homeOffset
    _Nim_get_homeOffset.restype = ctypes.c_int
    _Nim_get_homeOffset.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    offset = ctypes.c_double()
    nRes = _Nim_get_homeOffset(hMaster, nodeId, ctypes.byref(offset))
    return [nRes, offset.value]

'''
 * @brief 获取原点回归速度(6099:01/02)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity1 寻找开关速度（用户单位/s）
 * @param velocity2 寻找原点速度（用户单位/s）
 * @return 0 成功；其它 失败
 '''
def Nim_get_goHome_velocity(hMaster, nodeId):
    _Nim_get_goHome_velocity = SDKHandle.Nim_get_goHome_velocity
    _Nim_get_goHome_velocity.restype = ctypes.c_int
    _Nim_get_goHome_velocity.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]

    velocity1 = ctypes.c_double()
    velocity2 = ctypes.c_double()
    nRes = _Nim_get_goHome_velocity(hMaster, nodeId, ctypes.byref(velocity1), ctypes.byref(velocity2))
    return [nRes, velocity1.value, velocity2.value]

'''
 * @brief 获取原点回归加速度(609A)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param accel 加速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 '''
def Nim_get_goHome_accel(hMaster, nodeId):
    _Nim_get_goHome_accel = SDKHandle.Nim_get_goHome_accel
    _Nim_get_goHome_accel.restype = ctypes.c_int
    _Nim_get_goHome_accel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    accel = ctypes.c_double()
    nRes = _Nim_get_goHome_accel(hMaster, nodeId, ctypes.byref(accel))
    return [nRes, accel.value]

'''
 * @brief 设置原点偏移
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param offset 原点偏移，用户单位
 * @return 0 成功；其它 失败
 '''
def Nim_set_homeOffset(hMaster, nodeId, offset):
    _Nim_set_homeOffset = SDKHandle.Nim_set_homeOffset
    _Nim_set_homeOffset.restype = ctypes.c_int
    _Nim_set_homeOffset.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double]

    return _Nim_set_homeOffset(hMaster, nodeId, offset)

'''
  * @brief 设置原点回归速度(6099:01/02)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity1 寻找开关速度（用户单位/s）
 * @param velocity2 寻找原点速度（用户单位/s）
 * @return 0 成功；其它 失败
 '''
def Nim_set_goHome_velocity(hMaster, nodeId, velocity1, velocity2):
    _Nim_set_goHome_velocity = SDKHandle.Nim_set_goHome_velocity
    _Nim_set_goHome_velocity.restype = ctypes.c_int
    _Nim_set_goHome_velocity.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_double]

    return _Nim_set_goHome_velocity(hMaster, nodeId, velocity1, velocity2)
    
'''
 * @brief 设置原点回归加速度(609A)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param accel 加速度（用户单位/s^2）
 * @return 0 成功；其它 失败
 '''
def Nim_set_goHome_accel(hMaster, nodeId, accel):
    _Nim_set_goHome_accel = SDKHandle.Nim_set_goHome_accel
    _Nim_set_goHome_accel.restype = ctypes.c_int
    _Nim_set_goHome_accel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double]

    return _Nim_set_goHome_accel(hMaster, nodeId, accel)

'''
 * @brief 设置VM模式下的加速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param deltaV 速度变化量（rpm）
 * @param deltaT 时间变化量（S）
 *        加速度 = deltaV/deltaT
 * @return 0 成功；其它 失败
 '''
def Nim_set_vmAccel(hMaster, nodeId, deltaV, deltaT):
    _Nim_set_vmAccel = SDKHandle.Nim_set_vmAccel
    _Nim_set_vmAccel.restype = ctypes.c_int
    _Nim_set_vmAccel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32]

    return _Nim_set_vmAccel(hMaster, nodeId, deltaV, deltaT)

'''
 * @brief 设置VM模式下的减速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param deltaV 速度变化量（rpm）
 * @param deltaT 时间变化量（S）
 *        减速度 = deltaV/deltaT
 * @return 0 成功；其它 失败
 '''
def Nim_set_vmDecel(hMaster, nodeId, deltaV, deltaT):
    _Nim_set_vmDecel = SDKHandle.Nim_set_vmDecel
    _Nim_set_vmDecel.restype = ctypes.c_int
    _Nim_set_vmDecel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32]

    return _Nim_set_vmDecel(hMaster, nodeId, deltaV, deltaT)

'''
 * @brief 获取VM模式下的加速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param accel 输出参数，成功时返回加速度（rpm/s）
 * @return 0 成功；其它 失败
 '''
def Nim_get_vmAccel(hMaster, nodeId):
    _Nim_get_vmAccel = SDKHandle.Nim_get_vmAccel
    _Nim_get_vmAccel.restype = ctypes.c_int
    _Nim_get_vmAccel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    accel = ctypes.c_double()
    nRes = _Nim_get_vmAccel(hMaster, nodeId, ctypes.byref(accel))
    return [nRes,accel.value]

'''
 * @brief 获取VM模式下的减速度
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param accel 输出参数，成功时返回减速度（rpm/s）
 * @return 0 成功；其它 失败
 '''
def Nim_get_vmDecel(hMaster, nodeId):
    _Nim_get_vmDecel = SDKHandle.Nim_get_vmDecel
    _Nim_get_vmDecel.restype = ctypes.c_int
    _Nim_get_vmDecel.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    decel = ctypes.c_double()
    nRes = _Nim_get_vmDecel(hMaster, nodeId, ctypes.byref(decel))
    return [nRes,decel.value]

'''
 * @brief 通过6069获取当前速度
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return [nRes, speed]
 *          nRes 执行结果：0 成功；其它 失败
 *          velocity 执行成功时返回当前速度（用户单位/s）
 '''
def Nim_get_currentVelocity(hMaster, nodeId, bSDO):
    _Nim_get_currentVelocity = SDKHandle.Nim_get_currentVelocity
    _Nim_get_currentVelocity.restype = ctypes.c_int
    _Nim_get_currentVelocity.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]

    fVelocity = ctypes.c_double()
    nRes = _Nim_get_currentVelocity(hMaster, nodeId, ctypes.byref(fVelocity), bSDO)
    return [nRes, fVelocity.value]

'''
 * @brief 通过606C获取当前速度
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return [nRes, speed]
 *          nRes 执行结果：0 成功；其它 失败
 *          velocity 执行成功时返回当前速度（用户单位/s）
 '''
def Nim_get_currentVelocity2(hMaster, nodeId, bSDO):
    _Nim_get_currentVelocity2 = SDKHandle.Nim_get_currentVelocity2
    _Nim_get_currentVelocity2.restype = ctypes.c_int
    _Nim_get_currentVelocity2.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]

    fVelocity = ctypes.c_double()
    nRes = _Nim_get_currentVelocity2(hMaster, nodeId, ctypes.byref(fVelocity), bSDO)
    return [nRes, fVelocity.value]

'''
 * @brief 通过606C获取当前电机速度
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return [nRes, speed]
 *          nRes 执行结果：0 成功；其它 失败
 *          speed 执行成功时返回当前速度（rpm）
 '''
def Nim_get_currentMotorSpeed(hMaster, nodeId, bSDO):
    _Nim_get_currentMotorSpeed = SDKHandle.Nim_get_currentMotorSpeed
    _Nim_get_currentMotorSpeed.restype = ctypes.c_int
    _Nim_get_currentMotorSpeed.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int]

    nSpeed = ctypes.c_int()
    nRes = _Nim_get_currentMotorSpeed(hMaster, nodeId, ctypes.byref(nSpeed), bSDO)
    return [nRes, nSpeed.value]

'''
 * @brief 通过6064获取当前位置
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return [nRes, position]
 *          nRes 执行结果：0 成功；其它 失败
 *          position 执行成功时返回当前位置（用户单位u）
 '''
def Nim_get_currentPosition(hMaster, nodeId, bSDO):
    _Nim_get_currentPosition = SDKHandle.Nim_get_currentPosition
    _Nim_get_currentPosition.restype = ctypes.c_int
    _Nim_get_currentPosition.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]

    fPosition = ctypes.c_double()
    nRes = _Nim_get_currentPosition(hMaster, nodeId, ctypes.byref(fPosition), bSDO)
    return [nRes, fPosition.value]

'''
 * @brief 获取位置限制值(607D:01/02)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param minPos 输出参数，执行成功时返回最小极限位置
 * @param maxPos 输出参数，执行成功时返回最大极限位置
 * @return 0 成功；其它 失败
 '''
def Nim_get_posLimit(hMaster, nodeId):
    _Nim_get_posLimit = SDKHandle.Nim_get_posLimit
    _Nim_get_posLimit.restype = ctypes.c_int
    _Nim_get_posLimit.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]

    minPos = ctypes.c_double()
    maxPos = ctypes.c_double()
    nRes = _Nim_get_posLimit(hMaster, nodeId, ctypes.byref(minPos), ctypes.byref(maxPos))
    return [nRes, minPos.value, maxPos.value]

'''
 * @brief 设置位置限制值(607D:01/02)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param pos 输出参数，执行成功时返回最大极限位置
 * @return 0 成功；其它 失败
 '''
def Nim_set_posLimit(hMaster, nodeId,  minPos, maxPos):
    _Nim_set_posLimit = SDKHandle.Nim_set_posLimit
    _Nim_set_posLimit.restype = ctypes.c_int
    _Nim_set_posLimit.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double, ctypes.c_double]

    return _Nim_set_posLimit(hMaster, nodeId,  minPos, maxPos)

'''
  * @brief 获取最大速度(607F)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity 输出参数，成功时返回最大速度（用户单位/s）
 * @return 0 成功；其它 失败
 '''
def Nim_get_maxVelocity(hMaster, nodeId):
    _Nim_get_maxVelocity = SDKHandle.Nim_get_maxVelocity
    _Nim_get_maxVelocity.restype = ctypes.c_int
    _Nim_get_maxVelocity.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    velocity = ctypes.c_double()
    nRes = _Nim_get_maxVelocity(hMaster, nodeId, ctypes.byref(velocity))
    return [nRes, velocity.value]

'''
 * @brief 设置最大速度(607F)
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param velocity 最大速度（用户单位/s）
 * @return 0 成功；其它 失败
 '''
def Nim_set_maxVelocity(hMaster, nodeId, velocity):
    _Nim_set_maxVelocity = SDKHandle.Nim_set_maxVelocity
    _Nim_set_maxVelocity.restype = ctypes.c_int
    _Nim_set_maxVelocity.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double]

    return _Nim_set_maxVelocity(hMaster, nodeId, velocity)

'''
 * @brief 获取最大电机速度(6080)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque 输出参数，成功时返回最大电机速度（rpm）
 * @return 0 成功；其它 失败
 '''
def Nim_get_maxMotorSpeed(hMaster, nodeId):
    _Nim_get_maxMotorSpeed = SDKHandle.Nim_get_maxMotorSpeed
    _Nim_get_maxMotorSpeed.restype = ctypes.c_int
    _Nim_get_maxMotorSpeed.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]

    speed = ctypes.c_uint32()
    nRes = _Nim_get_maxMotorSpeed(hMaster, nodeId, ctypes.byref(speed))
    return [nRes, speed.value]

'''
 * @brief 设置最大电机速度(6080)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque  最大电机速度（rpm）
 * @return 0 成功；其它 失败
 '''
def Nim_set_maxMotorSpeed(hMaster, nodeId, speed):
    _Nim_set_maxMotorSpeed = SDKHandle.Nim_set_maxMotorSpeed
    _Nim_set_maxMotorSpeed.restype = ctypes.c_int
    _Nim_set_maxMotorSpeed.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32]

    return _Nim_set_maxMotorSpeed(hMaster, nodeId, speed)

'''
* @brief 获取最大转矩(6072)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque 输出参数，成功时返回最大转矩（0.001倍额定转矩）
 * @return 0 成功；其它 失败
 '''
def Nim_get_maxTorque(hMaster, nodeId):
    _Nim_get_maxTorque = SDKHandle.Nim_get_maxTorque
    _Nim_get_maxTorque.restype = ctypes.c_int
    _Nim_get_maxTorque.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]

    torque = ctypes.c_uint32()
    nRes = _Nim_get_maxTorque(hMaster, nodeId, ctypes.byref(torque))
    return [nRes, torque.value]

'''
  * @brief 设置最大转矩(6072)
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param torque  最大转矩（0.001倍额定转矩）
 * @return 0 成功；其它 失败
 '''
def Nim_set_maxTorque(hMaster, nodeId, torque):
    _Nim_set_maxTorque = SDKHandle.Nim_set_maxTorque
    _Nim_set_maxTorque.restype = ctypes.c_int
    _Nim_set_maxTorque.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32]

    return _Nim_set_maxTorque(hMaster, nodeId, torque)

'''
 * @brief 获取VM模式下的速度限制值
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param minSpeed 最小速度（rpm）
 * @param maxSpeed 最大速度（rpm）
 * @return 0 成功；其它 失败
 '''
def Nim_get_vmSpeedLimit(hMaster, nodeId):
    _Nim_get_vmSpeedLimit = SDKHandle.Nim_get_vmSpeedLimit
    _Nim_get_vmSpeedLimit.restype = ctypes.c_int
    _Nim_get_vmSpeedLimit.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32)]

    minSpeed = ctypes.c_uint32()
    maxSpeed = ctypes.c_uint32()
    nRes = _Nim_get_vmSpeedLimit(hMaster, nodeId, ctypes.byref(minSpeed), ctypes.byref(maxSpeed))
    return [nRes, minSpeed.value, maxSpeed.value]

'''
 * @brief 设置VM模式下的速度限制值
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param minSpeed 最小速度（rpm）
 * @param maxSpeed 最大速度（rpm）
 * @return 0 成功；其它 失败
 '''
def Nim_set_vmSpeedLimit(hMaster, nodeId, minSpeed, maxSpeed):
    _Nim_set_vmSpeedLimit = SDKHandle.Nim_set_vmSpeedLimit
    _Nim_set_vmSpeedLimit.restype = ctypes.c_int
    _Nim_set_vmSpeedLimit.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32]

    return _Nim_set_vmSpeedLimit(hMaster, nodeId, minSpeed, maxSpeed)

'''
 * @brief 设置用户单位的转换系数
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param factor 转换系数（电机编码器单位/用户单位）
 * @return 0 成功；其它 失败
 '''
def Nim_set_unitsFactor(hMaster, nodeId, factor):
    _Nim_set_unitsFactor = SDKHandle.Nim_set_unitsFactor
    _Nim_set_unitsFactor.restype = ctypes.c_int
    _Nim_set_unitsFactor.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_double]

    return _Nim_set_unitsFactor(hMaster, nodeId, factor)

'''
 * @brief 获取用户单位的转换系数
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param factor 转换系数（电机编码器单位/用户单位）
 * @return [nRes, Factor]
 *          nRes 执行结果：0 成功；其它 失败
 *          Factor 执行成功时返回转换系数（电机编码器单位/用户单位）
 '''
def Nim_get_unitsFactor(hMaster, nodeId):
    _Nim_get_unitsFactor = SDKHandle.Nim_get_unitsFactor
    _Nim_get_unitsFactor.restype = ctypes.c_int
    _Nim_get_unitsFactor.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]

    fFactor = ctypes.c_double()
    nRes = _Nim_get_unitsFactor(hMaster, nodeId, ctypes.byref(fFactor))
    return [nRes, fFactor.value]

'''
 * @brief  设置DO输出
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param nDOs DO值：每位表示一路DO：bit0表示DO1；bit1表示DO2，以此类推
 *                  1 输出高电平；0 输出低电平
 * @param bSDO 1 使用SDO输出；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_set_DOs(hMaster, nodeId, nDOs, bSDO):
    _Nim_set_DOs = SDKHandle.Nim_set_DOs
    _Nim_set_DOs.restype = ctypes.c_int
    _Nim_set_DOs.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32, ctypes.c_int]

    return _Nim_set_DOs(hMaster, nodeId, nDOs, bSDO)

'''
 * @brief  设置VDI值
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param nVDIs VDI值：每位表示一路VDI：bit0表示VDI1；bit1表示VDI2，以此类推
 *                  1 输出高电平；0 输出低电平
 * @return 0 成功；其它 失败
 '''
def Nim_set_VDIs(hMaster, nodeId, nVDIs):
    _Nim_set_VDIs = SDKHandle.Nim_set_VDIs
    _Nim_set_VDIs.restype = ctypes.c_int
    _Nim_set_VDIs.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32]

    return _Nim_set_VDIs(hMaster, nodeId, nVDIs)

'''
 * @brief  读取DI输入
 * @param hMaster 主站对象句柄
 * @param nodeId 轴地址
 * @param nDIs DI值：每位表示一路DI：bit0表示DI1；bit1表示DI2，以此类推
 *                  1 输入高电平；0 输入低电平
 * @param bSDO 1 使用SDO读取；0 使用PDO
 * @return 0 成功；其它 失败
 '''
def Nim_get_DIs(hMaster, nodeId, bSDO):
    _Nim_get_DIs = SDKHandle.Nim_get_DIs
    _Nim_get_DIs.restype = ctypes.c_int
    _Nim_get_DIs.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32), ctypes.c_int]

    nDIs = ctypes.c_uint32()
    nRes = _Nim_get_DIs(hMaster, nodeId, ctypes.byref(nDIs), bSDO)
    return [nRes, nDIs.value]

'''
 * @brief 保存所有参数到设备
 * @param hMaster 主站对象句柄
 * @param nodeId 从站地址
 * @param timeoutMS 超时时间，单位：ms
 * @return 0 成功；其它 失败
 '''
def Nim_save_AllParams(hMaster, nodeId, timeoutMS):
    _Nim_save_AllParams = SDKHandle.Nim_save_AllParams
    _Nim_save_AllParams.restype = ctypes.c_int
    _Nim_save_AllParams.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_int]

    return _Nim_save_AllParams(hMaster, nodeId, timeoutMS)

 
