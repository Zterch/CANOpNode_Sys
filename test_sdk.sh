#!/bin/bash
# 测试NiMotion SDK是否能正常工作

echo "=== Testing NiMotion SDK ==="

# 切换到SDK目录
cd /home/zterch/VS_Project/Nimo_COp_Prj/NimServoSDK-MM-bin-linux-x64/bin

echo "1. Checking USB device..."
lsusb | grep -i "NiMotion\|STMicro"

echo ""
echo "2. Running SDK test program..."
LD_LIBRARY_PATH=. ./test_csv 0 1001

echo ""
echo "=== Test completed ==="