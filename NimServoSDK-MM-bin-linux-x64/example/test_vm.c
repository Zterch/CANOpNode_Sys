
#include "nimservosdk.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef _WIN32
#include <Windows.h>
#define MySleepMS(msecs) Sleep(msecs)
#else
#include <unistd.h>
#include <string.h>
#define MySleepMS(msecs) usleep(msecs*1000)
#endif

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        printf("Usage: NimServoSDK_Test2 commType commParam [unitFactor]\r\n");
        return 0;
    }
    Nim_setLogFlags(1);
    int nVal = Nim_getLogFlags();
    printf("exec Nim_getLogFlags :%d\n", nVal);
    if (0 != Nim_init("./"))
    {
        printf("exec Nim_init faild\n");
        //return -1;
    }
    unsigned int hMaster;
    if (0 != Nim_create_master(atoi(argv[1]), &hMaster))
    {
        printf("exec Nim_create_master faild\n");
        goto EXIT_APP1;
    }
    if (atoi(argv[1]) < 0 || atoi(argv[1]) > 2)
        goto EXIT_APP2;

    char conn_str[200] = {0};
    switch (atoi(argv[1])) {
    case 0:     //CANopen
        switch (atoi(argv[2])) {
        case 1001:      //NiMotion USB-CAN Device
            sprintf(conn_str, "{\"DevType\": \"%s\", \"DevIndex\": 0, \"Baudrate\": 8,"\
                              " \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}", argv[2]);
            break;
        case 1002:      //Zhoulg USB-CAN Device
            sprintf(conn_str, "{\"DevType\": \"%s\", \"DevSubType\": 16, \"DevIndex\": 0, \"ChannelIndex\": 1,"\
                              " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}", argv[2]);
            break;
        case 1003:      //Ixxat USB-CAN Device
            sprintf(conn_str, "{\"DevType\": \"%s\", \"DevIndex\": 0, \"Baudrate\": 8,"\
                              " \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}", argv[2]);
            break;
        case 1004:      //NiMotion TCP-CAN Device
            sprintf(conn_str, "{\"DevType\": \"%s\", \"IP\": \"192.168.0.96\", \"Port\": 40001,"\
                              " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}", argv[2]);
            break;
        case 1005:      //Linux SocketCAN Device
            sprintf(conn_str, "{\"DevType\": \"%s\", \"DeviceName\": \"can0\","\
                              " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}", argv[2]);
            break;
        default:
            sprintf(conn_str, "{\"DevType\": \"%s\", \"DevIndex\": 0, \"Baudrate\": 8,"\
                              " \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}", "1001");
            break;
        }
        break;
    case 1:     //EtherCAT
        sprintf(conn_str, "{\"NetworkAdapter\": \"%s\", \"OverlappingPDO\": true, \"PDOIntervalMS\": 5}", argv[2]);
        break;
    case 2:     //Modbus
        sprintf(conn_str, "{\"SerialPort\": \"%s\", \"Baudrate\": 115200, \"Parity\": \"N\", \"DataBits\": 8, \"StopBits\": 1,"\
                          " \"PDOIntervalMS\": 20, \"SyncIntervalMS\": 0}", argv[2]);
        break;
    default:
        break;
    }

    printf("%s\r\n", conn_str);

    if (0 != Nim_master_run(hMaster, conn_str))
    {
        printf("exec Nim_master_run faild\n");
        goto EXIT_APP2;
    }
    Nim_master_changeToPreOP(hMaster);
    MySleepMS(50);              // 必要延时(Nim_master_changeToPreOP后)

    int nAddrs[10] = {0};
    int i = 0, nSlaveCount = 0;
    Nim_scan_nodes(hMaster, 1, 10);
    for (i=0; i<10; i++)
    {
        if (0 != Nim_is_online(hMaster, i))
        {
            nAddrs[nSlaveCount] = i;
            printf("motor %d is online\n", i);
            nSlaveCount++;
        }
    }
    if (nSlaveCount == 0)
    {
        printf("There is no motor online\n");
        goto EXIT_APP3;
    }

    double fUnitFactor = 10000.0;
    if (argc > 3)
    {
        fUnitFactor = atof(argv[3]);
    }
    printf("UnitFactor = %f\r\n", fUnitFactor);

    int nRe = 0;
    for (int nIndex = 0; nIndex < nSlaveCount; nIndex++)
    {
        if (atoi(argv[1]) == 2)
            nRe = Nim_load_params(hMaster, nAddrs[nIndex], "Modbus.db");
        else if (atoi(argv[1]) == 1)
            nRe = Nim_load_params(hMaster, nAddrs[nIndex], "EtherCAT.db");
        else
            nRe = Nim_load_params(hMaster, nAddrs[nIndex], "CANopen.db");

        if (0 != nRe)
        {
            printf("exec Nim_load_params failed\r\n");
            goto EXIT_APP3;
        }

        nRe = Nim_read_PDOConfig(hMaster, nAddrs[nIndex]);
        if (0 != nRe)
        {
            printf("exec Nim_read_PDOConfig failed\r\n");
            goto EXIT_APP3;
        }
    }
    nRe = Nim_master_changeToOP(hMaster);
    MySleepMS(200);          // 必要延时(Nim_master_changeToOP后)
    for (int nIndex = 0; nIndex < nSlaveCount; nIndex++)
    {
        nRe = Nim_set_unitsFactor(hMaster, nAddrs[nIndex], fUnitFactor);
        nRe = Nim_clearError(hMaster, nAddrs[nIndex], 1);
        nRe = Nim_power_off(hMaster, nAddrs[nIndex], 1);
        MySleepMS(50);      // 必要延时(Nim_power_off后)
        nRe = Nim_set_workMode(hMaster, nAddrs[nIndex], SERVO_VM_MODE, 1);
        MySleepMS(50);      // 必要延时(Nim_set_workMode后)
        nRe = Nim_set_vmTargetSpeed(hMaster, nAddrs[nIndex], 0, 1);
        nRe = Nim_set_vmSpeedLimit(hMaster, nAddrs[nIndex], 0, 500); 
        nRe = Nim_set_vmAccel(hMaster, nAddrs[nIndex], 200, 1);
        nRe = Nim_set_vmDecel(hMaster, nAddrs[nIndex], 200, 1);
        nRe = Nim_power_on(hMaster, nAddrs[nIndex], 1);
    }
    MySleepMS(200);          // 必要延时(Nim_power_on后)
    // 控制字发0x7F控制电机在VM模式下运行
    nRe = Nim_set_controlWord(hMaster, nAddrs[0], 0x7F, 1);
    printf("********************run backward*********************\n");
    nRe = Nim_set_vmTargetSpeed(hMaster, nAddrs[0], 200, 1);
    double vmSpeed = 0;
    unsigned short statusWord = 0;
    do
    {
        MySleepMS(50);
        if (0 == Nim_get_currentVelocity2(hMaster, nAddrs[0], &vmSpeed, 0)
                && 0 == Nim_get_statusWord(hMaster, nAddrs[0], &statusWord, 0))
            printf("statusword = %04x \t currentSpeed = %f rpm\t\r", statusWord, vmSpeed);
			fflush(stdout);
    }while ((statusWord & 0x400) == 0);
    printf("\n");
    MySleepMS(4000);

    printf("********************run backward*********************\n"); 
    nRe = Nim_set_vmTargetSpeed(hMaster, nAddrs[0], -200, 1);
    do 
    {
        MySleepMS(50);
        if (0 == Nim_get_currentVelocity2(hMaster, nAddrs[0], &vmSpeed, 0)
                && 0 == Nim_get_statusWord(hMaster, nAddrs[0], &statusWord, 0))
            printf("statusword = %04x \t currentSpeed = %f rpm\t\r", statusWord, vmSpeed);
			fflush(stdout);
    }while ((statusWord & 0x400) == 0);
    printf("\n");
    MySleepMS(4000);

    nRe = Nim_set_vmTargetSpeed(hMaster, nAddrs[0], 0, 1);
    do
    {
        MySleepMS(50);
        if (0 == Nim_get_currentVelocity2(hMaster, nAddrs[0], &vmSpeed, 0)
                && 0 == Nim_get_statusWord(hMaster, nAddrs[0], &statusWord, 0))
            printf("statusword = %04x \t currentSpeed = %f rpm\t\r", statusWord, vmSpeed);
			fflush(stdout);
    }while ((statusWord & 0x400) == 0);
    printf("\n");

    for (int nIndex = 0; nIndex < nSlaveCount; nIndex++)
    {
        nRe = Nim_power_off(hMaster, nAddrs[nIndex], 1);
    }
    MySleepMS(50);          // 必要延时(Nim_power_off后)
    nRe = Nim_master_changeToPreOP(hMaster);
    MySleepMS(50);          // 必要延时(Nim_master_changeToPreOP后)
EXIT_APP3:
    nRe = Nim_master_stop(hMaster);
EXIT_APP2:
    Nim_destroy_master(hMaster);
EXIT_APP1:
    Nim_clean();
    return 0;
}
