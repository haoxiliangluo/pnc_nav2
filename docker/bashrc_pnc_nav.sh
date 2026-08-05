# pnc_nav2 workspace environment (appended into container ~/.bashrc)

source /opt/ros/jazzy/setup.bash

if [ -d /opt/ros/jazzy/opt/gz_tools_vendor/bin ]; then
  export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:${PATH}"
fi

if [ -f /home/hao/pnc_nav2/install/setup.bash ]; then
  source /home/hao/pnc_nav2/install/setup.bash
fi

cd /home/hao/pnc_nav2 2>/dev/null || true
