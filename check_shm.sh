#!/bin/bash
# 共享内存诊断脚本

echo "=== 共享内存诊断 ==="
echo ""

# 检查下位机进程
echo "1. 检查下位机进程:"
ps aux | grep CANOpNode_Sys | grep -v grep
if [ $? -eq 0 ]; then
    echo "   ✓ 下位机正在运行"
else
    echo "   ✗ 下位机未运行"
fi
echo ""

# 检查共享内存文件
echo "2. 检查共享内存文件:"
for path in /dev/shm/gravshow_shm /run/shm/gravshow_shm; do
    if [ -e "$path" ]; then
        echo "   找到: $path"
        ls -la "$path"
        echo "   权限: $(stat -c '%a' $path)"
    fi
done
echo ""

# 检查当前用户
echo "3. 当前用户信息:"
echo "   用户: $(whoami)"
echo "   UID: $(id -u)"
echo "   GID: $(id -g)"
echo ""

# 尝试访问共享内存
echo "4. 测试共享内存访问:"
if [ -e "/dev/shm/gravshow_shm" ]; then
    if [ -r "/dev/shm/gravshow_shm" ]; then
        echo "   ✓ 可读"
    else
        echo "   ✗ 不可读"
    fi
    if [ -w "/dev/shm/gravshow_shm" ]; then
        echo "   ✓ 可写"
    else
        echo "   ✗ 不可写"
    fi
else
    echo "   共享内存不存在"
fi
