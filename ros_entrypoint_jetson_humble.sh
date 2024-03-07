#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash" --
source "/root/ros2_ws/install/local_setup.bash" --

# Welcome information
echo "ZED ROS2 Docker Image"
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo 'DDS middleware: ' $RMW_IMPLEMENTATION 
echo "---"  
echo 'Available ZED packages:'
# ros2 pkg list | grep zed
echo "---------------------"    

# Run your command in the background
# ros2 run tf2_ros static_transform_publisher 0 0.06 0 0 0 0 base_link zed2i_camera_center &
# ros2 launch zed_wrapper zed_camera.launch.py camera_model:='zed2i' camera_name:='zed'

exec "$@"
