#!/bin/bash
# 一键启动 roscore + arm + d435
# 用法: bash start_all_nodes.sh

while [ -n "$CONDA_PREFIX" ]; do
    conda deactivate 2>/dev/null || break
done
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "=== 启动 ROS 节点 ==="

# 1. roscore
echo "[1/3] roscore..."
roscore &
sleep 3

# 2. 双臂 driver
echo "[2/3] 双臂 driver..."
roslaunch embodied_arm_driver dual_arm_65_driver.launch Udp_hand:=true &
sleep 5

# 3. D435 相机
echo "[3/3] D435 相机..."
roslaunch embodied_camera d435_camera_pub.launch &
sleep 3

echo ""
echo "=== 全部启动 ==="
echo "  roscore + arm_driver + D435"
echo ""
echo "停全部: killall -9 roscore roslaunch"
wait
