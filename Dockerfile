FROM isaac_ros_dev-aarch64
ENV DEBIAN_FRONTEND=noninteractive
# uodate and install dependencies 
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl \
    ros-humble-realsense2-camera \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-occupancy-grid-localizer\
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Cheange the ROS2 RMW to CycloneDDS as instructed by Unitree
RUN cd / && git clone https://github.com/unitreerobotics/unitree_ros2 && cd /unitree_ros2/cyclonedds_ws/src && \
git clone https://github.com/ros2/rmw_cyclonedds -b humble && git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x &&\
cd .. && colcon build --packages-select cyclonedds && source /opt/ros/humble/setup.bash && colcon build

RUN echo "source /opt/ros/humble/setup.bash" >>  /usr/local/bin/scripts/workspace-entrypoint.sh
RUN echo "source /unitree_ros2/cyclonedds_ws/install/setup.bash" >>  /usr/local/bin/scripts/workspace-entrypoint.sh
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >>  /usr/local/bin/scripts/workspace-entrypoint.sh
RUN echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces> <NetworkInterface name="eth0" priority="default" multicast="default" />  </Interfaces></General></Domain></CycloneDDS>'" >>  /usr/local/bin/scripts/workspace-entrypoint.sh

# copy the go2py ros2 nodes
COPY deploy/ros2_ws/src /ros2_ws/src
RUN cd /ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Compile the C++ hypervisor bridge
COPY cpp_bridge /cpp_bridge
WORKDIR /cpp_bridge
RUN ./install.sh && mkdir build && cd build && cmake .. && make

# Copy the script to start the nodes
COPY deploy/scripts /root/scripts
COPY deploy/launch /root/launch
# set the entrypoint to bash
# ENTRYPOINT ["/bin/bash"]
ENTRYPOINT ["/bin/bash", "/root/scripts/hw_start.sh"]
