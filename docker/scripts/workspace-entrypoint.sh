#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
sudo service udev restart

cd ~
source /realsense-ws/install/setup.bash
#source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
#                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
#                        </Interfaces></General></Domain></CycloneDDS>'
$@
