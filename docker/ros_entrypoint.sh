#!/bin/bash
set -e

# Source ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Source CAVERS workspace
if [ -f /root/cavers_ws/install/local_setup.bash ]; then
  source /root/cavers_ws/install/local_setup.bash
fi

exec "$@"