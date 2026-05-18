#!/bin/bash
# 运行CANOpNode_Sys并设置正确的环境变量
# 关键：sudo -E 保留环境变量

SDK_PATH="/home/zterch/VS_Project/Nimo_COp_Prj/NimServoSDK-MM-bin-linux-x64/bin"

echo "=== CANOpNode_Sys Launcher ==="
echo "SDK Path: $SDK_PATH"
echo ""

# 设置环境变量并运行（使用sudo -E保留环境变量）
export LD_LIBRARY_PATH="$SDK_PATH:$LD_LIBRARY_PATH"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo ""

# 使用sudo -E保留环境变量运行
sudo -E ./bin/CANOpNode_Sys