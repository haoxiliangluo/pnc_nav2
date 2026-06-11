#!/bin/bash
# 启动后手动激活map_server

cd /home/hao/pnc_nav2
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=== 启动系统 ==="
ros2 launch pnc_nav_bringup turtlebot3_test.launch.py &
LAUNCH_PID=$!

echo "等待节点启动..."
sleep 10

echo "=== 激活map_server ==="
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

echo ""
echo "=== 系统就绪 ==="
echo "- 在RViz中使用 '2D Goal Pose' 设置目标"
echo "- 或运行: ros2 topic pub --once /goal_pose ..."
echo ""
echo "按Ctrl+C退出"
wait $LAUNCH_PID
