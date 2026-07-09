#!/bin/sh

# 预设检测时长
ALL_TIME=60 # s
# 最小时常，等待机械臂连接
MIN_TIME=20 # s
# 设备数量
DEVICE_NUM=4

# 设置启动末端执行器的脚本
END_EFFECTORS=("brain_co_ctrl" "oy_motion_ctrl" "zhixing_ctrl" "dhag_grip_ctrl")
END_EFFECTOR=${END_EFFECTORS[2]}

# ip组"左臂 右臂 底盘"
IP_LIST="169.254.128.18 169.254.128.19 169.254.128.2"
echo "开始检测IP连通性..."

# 最大尝试ping设备ip次数
MAX_ATTEMPTS=30
# 每次连接堵塞时常
IP_INTERVAL=1
# 是否ping通所有ip
all_reachable=false
# 尝试次数
attempt=0

# 循环ping
while [ "$all_reachable" = false ] && [ $attempt -lt $MAX_ATTEMPTS ]; do
    all_reachable=true
    attempt=$((attempt+1))
    clear
    
    echo "尝试 $((attempt))/$MAX_ATTEMPTS:"
    echo "------------------------"
    
    # 改用 for in $var 语法
    for ip in $IP_LIST; do
        if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
            echo "[✓] $ip 可达"
        else
            echo "[×] $ip 不可达"
            all_reachable=false
        fi
    done
    
    [ "$all_reachable" = true ] && break
    
    echo "------------------------"
    echo "等待 $IP_INTERVAL 秒后重试..."
    sleep $IP_INTERVAL
done

[ "$all_reachable" = false ] && exit 1

WAIT_INTERVAL=2 # 设备单次检测秒数, sudo udevadm trigger耗时s
MAX_WAIT=$(( (ALL_TIME - attempt * IP_INTERVAL) / WAIT_INTERVAL ))
echo $MAX_WAIT
count=0
trigger_will_times=1

# 循环等待所有设备
all_found=false
while [ $count -lt $MAX_WAIT ]; do
    
    RMUSB_COUNT=$(ls -l /dev | grep ttyUSB | grep rmUSB 2>/dev/null | wc -l)

    # 检查设备
    if [ "$RMUSB_COUNT" -eq $DEVICE_NUM ]; then
        all_found=true
        break
    else
        sudo udevadm trigger  # 开始刷新规则
        sudo udevadm settle  # 等待结束
    fi
    echo "Waiting for ports... ($count/$MAX_WAIT)"
    count=$((count + 1))

    sleep $WAIT_INTERVAL
done

# 等机械臂初始化成功
sleep $(( (MIN_TIME - attempt * IP_INTERVAL - WAIT_INTERVAL * count) > 0 ? (MIN_TIME - attempt * IP_INTERVAL - WAIT_INTERVAL * count) : 0 ))

# 判断结果
if [ "$all_found" = true ]; then
    echo "All devices found! Proceeding..."
    
    # 启动 atom
    gnome-terminal --tab --title="atom" -- bash -c "
        cd ~/rmc_aida_l_atom/build || exit 1
        ./atom || {
            echo 'Failed to start atom'
            read -p 'Press enter to close...'
        }
        exec bash
    "
    sleep 3
    # 启动 head servo ctrl
    gnome-terminal --tab --title="head_servo" -- bash -c "
        cd ~/rmc_aida_l_atom/scripts || exit 1
        python3 head_servo_ctrl.py || {
            echo 'Failed to start head servo ctrl'
            read -p 'Press enter to close...'
        }
        exec bash
    "
    sleep 3
    # 启动 effector
    gnome-terminal --tab --title="effector" -- bash -c "
        cd ~/rmc_aida_l_atom/scripts || exit 1
        python3 $END_EFFECTOR.py || {
            echo 'Failed to start $END_EFFECTOR.py'
            read -p 'Press enter to close...'
        }
        exec bash
    "

else
    echo "Error: the following ports not found after $MAX_WAIT seconds:"
    exit 1
fi
