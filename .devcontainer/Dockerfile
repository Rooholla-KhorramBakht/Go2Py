FROM ros:foxy
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y -qq --no-install-recommends \ 
                                      libglvnd-dev \
                                      libgl1-mesa-dev \
                                      libegl1-mesa-dev \
                                      libgles2-mesa-dev \
                                      libxext6 \
                                      libx11-6 \
                                      freeglut3-dev \
                                      git \
                                      python3-pip \ 
                                      ros-foxy-rmw-cyclonedds-cpp ros-foxy-rosidl-generator-dds-idl \
                                      libyaml-cpp-dev \
                                      ros-foxy-xacro \
                                      libboost-all-dev\
                                      build-essential \
                                      cmake \
                                      && rm -rf /var/lib/apt/lists/*


RUN pip3 install mujoco pin matplotlib

RUN cd / && git clone https://github.com/unitreerobotics/unitree_ros2 && cd /unitree_ros2/cyclonedds_ws/src && \
git clone https://github.com/ros2/rmw_cyclonedds -b foxy && git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x &&\
cd .. && colcon build --packages-select cyclonedds && source /opt/ros/foxy/setup.bash && colcon build

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute