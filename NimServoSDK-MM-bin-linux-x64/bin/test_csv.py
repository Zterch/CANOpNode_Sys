#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import ctypes, os, types, sys, time, math
from NimServoSDK import *
 
Nim_setLogFlags(1)
val = Nim_getLogFlags()
print(val)
# 使用相对路径初始化SDK（与C版本相同）
sdk_path = '/home/zterch/VS_Project/Nimo_COp_Prj/NimServoSDK-MM-bin-linux-x64/bin'
# 先尝试使用相对路径
Nim_init('./')  
[res, hMaster] = Nim_create_master(0)   #0-CANopen 1-EtherCAT 2-Modbus
if res == 0:
    #CANopen - 使用Linux SocketCAN (PCAN)
    # DevType 1005 = Linux SocketCAN Device
    # DeviceName can0 = SocketCAN接口名称
    # Baudrate 8 = 1000Kbps (在SocketCAN中通过ip link设置波特率)
    Nim_master_run(hMaster, '{"DevType": "1005", "DeviceName": "can0", "Baudrate": 8, "PDOIntervalMS": 10, "SyncIntervalMS": 10}')
    #Nim_master_run(hMaster, '{"DevType": "1002", "DevSubType": 16, "DevIndex": 0, "ChannelIndex": 1, "Baudrate": 8, "PDOIntervalMS": 10, "SyncIntervalMS": 10}')
    #Nim_master_run(hMaster, '{"DevType": "1003", "DevIndex": 0, "Baudrate": 8, "PDOIntervalMS": 10, "SyncIntervalMS": 10}')
    #Nim_master_run(hMaster, '{"DevType": "1004", "IP": "192.168.0.96", "Port": 40001, "Baudrate": 8, "PDOIntervalMS": 10, "SyncIntervalMS": 10}')
    #Nim_master_run(hMaster, '{"DevType": "1005", "DeviceName": "can0", "Baudrate": 6, "PDOIntervalMS": 10, "SyncIntervalMS": 10}')  
    #EtherCAT
    #Nim_master_run(hMaster, '{"NetworkAdapter": "\\\\Device\\\\NPF_{049FF630-1879-4F9A-BD9B-A8B4875294BC}", "OverlappingPDO": true, "PDOIntervalMS": 10}')
    #Nim_master_run(hMaster, '{"NetworkAdapter": "enp2s0", "OverlappingPDO": true, "PDOIntervalMS": 10}')
    #Modbus
    #Nim_master_run(hMaster, '{"SerialPort": "COM8", "Baudrate": 115200, "Parity": "N", "DataBits": 8, "StopBits": 1, "PDOIntervalMS": 20, "SyncIntervalMS": 0}')
    #Nim_master_run(hMaster, '{"SerialPort": "/dev/ttyUSB0", "Baudrate": 115200, "Parity": "N", "DataBits": 8, "StopBits": 1, "PDOIntervalMS": 20, "SyncIntervalMS": 0}')
 
    Nim_master_changeToPreOP(hMaster);
    time.sleep(0.05)            # 必要延时(Nim_master_changeToPreOP后)
    Nim_scan_nodes(hMaster, 1, 10)
    nCurAddr = 0
    for addr in range(1, 10):
        if 1 == Nim_is_online(hMaster, addr):
            nCurAddr = addr
            print("motor %d is online\n" % addr)
            break; 

    if nCurAddr > 0:
        # 使用绝对路径加载参数文件
        import os
        db_path = os.path.join(sdk_path, "CANopen.db")
        Nim_load_params(hMaster, nCurAddr, db_path)
        #Nim_load_params(hMaster, nCurAddr, "Modbus.db")
        #Nim_load_params(hMaster, nCurAddr, "EtherCAT.db")
        Nim_read_PDOConfig(hMaster, nCurAddr)
        Nim_set_unitsFactor(hMaster, nCurAddr, 10000.0)
        #Nim_set_unitsFactor(hMaster, nCurAddr, 1600.0)
        Nim_clearError(hMaster, nCurAddr, 1)
        Nim_master_changeToOP(hMaster)
        time.sleep(0.05)     # 必要延时(Nim_master_changeToOP后)
        Nim_power_off(hMaster, nCurAddr, 1)
        time.sleep(0.05)     # 必要延时(Nim_power_off后)
        Nim_set_workMode(hMaster, nCurAddr, ServoWorkMode.SERVO_CSV_MODE, 1)
        time.sleep(0.05)     # 必要延时(Nim_set_workMode)
        Nim_power_on(hMaster, nCurAddr, 0)
        time.sleep(0.2)      # 必要延时(Nim_power_on后)
        
        print("********************move csv*********************\n")
        #time.sleep(1.0)
        t = 0.0
        while True:
            fVelocity = 5.0 * math.sin(2.0*math.pi/5.0*t)
            Nim_set_targetVelocity(hMaster, nCurAddr, fVelocity, 0)
            time.sleep(0.01)
            t = t + 0.01
            [nRes, sw] = Nim_get_statusWord(hMaster, nCurAddr, 0)
            [nRes, pos] = Nim_get_currentPosition(hMaster, nCurAddr, 0)
            [nRes, vel] = Nim_get_currentVelocity(hMaster, nCurAddr, 0)
            print("result: %d, status_word: %d, position: %f, velocity: %f" % (nRes, sw, pos, vel))
            if t > 10.0:
                break

        Nim_power_off(hMaster, nCurAddr, 1)
        time.sleep(0.05)    # 必要延时(Nim_power_off后)
        Nim_master_changeToPreOP(hMaster)
        time.sleep(0.05)    # 必要延时(Nim_master_changeToPreOP后)
    Nim_master_stop(hMaster)
    Nim_destroy_master(hMaster)
Nim_clean()
