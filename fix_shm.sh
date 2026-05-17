#!/bin/bash
# 修复共享内存权限问题的完整脚本

echo "=== 修复共享内存权限 ==="
echo ""

# 1. 清理旧共享内存
echo "1. 清理旧共享内存..."
sudo rm -f /dev/shm/gravshow_shm
sudo rm -f /run/shm/gravshow_shm
echo "   完成"
echo ""

# 2. 检查下位机是否在运行
echo "2. 检查下位机进程..."
PID=$(ps aux | grep CANOpNode_Sys | grep -v grep | awk '{print $2}')
if [ ! -z "$PID" ]; then
    echo "   下位机正在运行 (PID=$PID)，停止它..."
    sudo kill $PID 2>/dev/null
    sleep 1
fi
echo "   完成"
echo ""

# 3. 启动下位机
echo "3. 启动下位机..."
echo "   等待下位机启动并创建共享内存..."
cd /home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys
sudo ./bin/CANOpNode_Sys &
PID=$!
sleep 3

# 4. 检查共享内存权限
echo ""
echo "4. 检查共享内存权限..."
if [ -e "/dev/shm/gravshow_shm" ]; then
    PERM=$(stat -c '%a' /dev/shm/gravshow_shm)
    OWNER=$(stat -c '%U' /dev/shm/gravshow_shm)
    echo "   共享内存: /dev/shm/gravshow_shm"
    echo "   权限: $PERM"
    echo "   所有者: $OWNER"
    
    if [ "$PERM" = "666" ]; then
        echo "   ✓ 权限正确 (666)"
    else
        echo "   ✗ 权限错误 (应为666)"
        echo "   尝试修复..."
        sudo chmod 666 /dev/shm/gravshow_shm
        echo "   修复后权限: $(stat -c '%a' /dev/shm/gravshow_shm)"
    fi
else
    echo "   ✗ 共享内存未创建"
fi
echo ""

echo "=== 修复完成 ==="
echo "下位机PID: $PID"
echo "现在可以启动上位机了:"
echo "  cd /home/zterch/VS_Project/Nimo_COp_Prj/GravShow && ./bin/GravShow"
echo ""
echo "按Ctrl+C停止下位机"
wait $PID
