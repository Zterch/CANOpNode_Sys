#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import ctypes, os, types, sys, time
from NimServoSDK import *
 
Nim_setLogFlags(1)
val = Nim_getLogFlags()
print(val)
#Nim_init('/usr/local/NimServoSDK-MM-bin-linux-arm_v7a/bin')
Nim_init(r'D:\03-SDK\NiMServoSDK-MM_2023\NimServoSDK-MM-RF-bin-Windows-X86\bin')  
[res, hMaster] = Nim_create_master(0)   #0-CANopen 1-EtherCAT 2-Modbus
if res == 0:
    #CANopen
    Nim_master_run(hMaster, '{"DevType": "1001", "DevIndex": 0, "Baudrate": 8, "PDOIntervalMS": 10, "SyncIntervalMS": 10}')
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
        Nim_load_params(hMaster, nCurAddr, "CANopen.db")
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
        Nim_set_workMode(hMaster, nCurAddr, ServoWorkMode.SERVO_PT_MODE, 1)
        time.sleep(0.05)     # 必要延时(Nim_set_workMode)
        Nim_set_PT_SpeedLimit(hMaster, nCurAddr, 500, 500);
        time.sleep(0.1)
        Nim_set_PT_TorqueRamp(hMaster, nCurAddr, 0);
        time.sleep(0.1)
        Nim_set_targetTorque(hMaster, nCurAddr, 200, 1);
        time.sleep(0.1) 
        Nim_power_on(hMaster, nCurAddr, 1)
        time.sleep(5) 

        Nim_power_off(hMaster, nCurAddr, 1)
        time.sleep(0.05)    # 必要延时(Nim_power_off后)
        Nim_master_changeToPreOP(hMaster)
        time.sleep(0.05)    # 必要延时(Nim_master_changeToPreOP后)
    Nim_master_stop(hMaster)
    Nim_destroy_master(hMaster)
Nim_clean()
