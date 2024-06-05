#!/bin/bash

set -e
apt-get install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl 

cd ~
git clone https://github.com/unitreerobotics/unitree_ros2
cd unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd .. 
colcon build --packages-select cyclonedds
source /opt/ros/humble/setup.bash 
colcon build
