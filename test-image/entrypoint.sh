#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/ros2_ws/install/setup.sh"

exec "$@"
