#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"
