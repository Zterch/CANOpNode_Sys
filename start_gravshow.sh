#!/bin/bash
# 上位机启动脚本

echo "=== 上位机启动脚本 ==="
echo ""

# 检查共享内存是否存在
echo "1. 检查共享内存..."
if [ -e "/dev/shm/gravshow_shm" ]; then
    echo "   ✓ 共享内存存在"
    ls -la /dev/shm/gravshow_shm
else
    echo "   ✗ 共享内存不存在"
    echo "   请先启动下位机！"
    exit 1
fi
echo ""

# 检查权限
echo "2. 检查权限..."
if [ -r "/dev/shm/gravshow_shm" ] && [ -w "/dev/shm/gravshow_shm" ]; then
    echo "   ✓ 权限正常"
else
    echo "   ✗ 权限不足，尝试修复..."
    sudo chmod 666 /dev/shm/gravshow_shm
    if [ -r "/dev/shm/gravshow_shm" ] && [ -w "/dev/shm/gravshow_shm" ]; then
        echo "   ✓ 权限修复成功"
    else
        echo "   ✗ 权限修复失败"
        exit 1
    fi
fi
echo ""

# 启动上位机
echo "3. 启动上位机 (GravShow)..."
cd /home/zterch/VS_Project/Nimo_COp_Prj/GravShow
./bin/GravShow

echo ""
echo "4. 上位机已退出"
