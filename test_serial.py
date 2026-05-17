import serial
import serial.tools.list_ports

# 列出所有可用串口
print("可用串口:")
ports = serial.tools.list_ports.comports()
for port in ports:
    print(f"- {port.device}: {port.description}")

# 尝试打开第一个串口
if ports:
    port = ports[0].device
    print(f"\n尝试打开串口: {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"成功打开串口: {port}")
        ser.close()
    except Exception as e:
        print(f"打开串口失败: {e}")
else:
    print("没有找到可用串口")
