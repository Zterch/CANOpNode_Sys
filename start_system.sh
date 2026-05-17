#!/bin/bash
# 重力卸载系统启动脚本

echo "=== 重力卸载系统启动脚本 ==="
echo ""

# 清理旧共享内存
echo "1. 清理旧共享内存..."
sudo rm -f /dev/shm/gravshow_shm
sudo rm -f /run/shm/gravshow_shm
echo "   完成"
echo ""

# 启动下位机
echo "2. 启动下位机 (CANOpNode_Sys)..."
echo "   请在下位机启动后，再启动上位机"
echo ""
cd /home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys
sudo ./bin/CANOpNode_Sys

# 下位机退出后清理
echo ""
echo "3. 下位机已退出，清理共享内存..."
sudo rm -f /dev/shm/gravshow_shm
