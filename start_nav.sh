#!/bin/bash
# Launch Gazebo Harmonic playground + maps/111 + nav_server

set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"

# === 容器隔离：避免干扰其他容器的仿真（trash-sim: domain 42 / GZ_PARTITION=trashbot） ===
export ROS_DOMAIN_ID=${PNC_NAV_DOMAIN_ID:-43}
export GZ_PARTITION=${PNC_NAV_GZ_PARTITION:-pnc_nav}

# Gazebo Harmonic CLI (ros_gz vendor layout on some installs)
if [ -d /opt/ros/jazzy/opt/gz_tools_vendor/bin ]; then
  export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:${PATH}"
fi

echo "=== 启动系统 (Gazebo Sim + playground + maps/111) ==="
ros2 launch pnc_nav_bringup gz_sim_2d_bringup.launch.py &
LAUNCH_PID=$!

echo "等待节点启动..."
sleep 15

echo ""
echo "=== 系统就绪 ==="
echo "- RViz 中使用 '2D Goal Pose' 设置目标"
echo "- 检查: ros2 topic echo /map --once"
echo "- 检查: ros2 topic echo /odom --once"
echo ""
echo "按Ctrl+C退出"
wait $LAUNCH_PID
