#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/foxy/setup.bash"

echo "==============LIO-SAM ROS2 Docker Env Ready================"

cd /root/ros2_ws

exec "$@"

