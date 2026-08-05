#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

if [ -d /opt/ros/jazzy/opt/gz_tools_vendor/bin ]; then
  export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:${PATH}"
fi

if [ -f /home/hao/pnc_nav2/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/hao/pnc_nav2/install/setup.bash
fi

cd /home/hao/pnc_nav2 2>/dev/null || true

exec "$@"
