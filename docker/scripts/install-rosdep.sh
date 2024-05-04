#!/bin/bash

set -e

# Make sure workspace is mounted in /tmp/ros_ws
source /opt/ros/humble/setup.bash
apt-get update
rosdep init
rosdep update
rosdep install --from-paths /tmp/ros_ws --ignore-src -y