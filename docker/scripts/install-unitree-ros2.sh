#!/bin/bash

set -e

# clone unitree_ros2
cd /workspace/go2-devcontainer/src
git clone https://github.com/unitreerobotics/unitree_ros2

# clone cyclonedds related packaged
cd /workspace/go2-devcontainer/src/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 

# build cyclonedds
cd /workspace/go2-devcontainer/src/unitree_ros2/cyclonedds_ws
colcon build --packages-select cyclonedds