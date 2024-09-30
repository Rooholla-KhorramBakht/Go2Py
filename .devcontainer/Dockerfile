FROM ros:humble
ENV DEBIAN_FRONTEND noninteractive
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
                                      python3-tk \
                                      curl \
                                      vim \ 
                                      libcgal-dev \
                                      libcgal-demo \
                                      ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl \
                                      libyaml-cpp-dev \
                                      ros-humble-xacro \
                                      libboost-all-dev\
                                      build-essential \
                                      cmake \
                                      git \
                                      && rm -rf /var/lib/apt/lists/*
                                      
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
RUN apt-get install git-lfs

# Cheange the ROS2 RMW to CycloneDDS as instructed by Unitree
RUN cd / && git clone https://github.com/unitreerobotics/unitree_ros2 && cd /unitree_ros2/cyclonedds_ws/src && \
git clone https://github.com/ros2/rmw_cyclonedds -b humble && git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x &&\
cd .. && colcon build --packages-select cyclonedds && source /opt/ros/humble/setup.bash && colcon build

# Install Python dependencies
RUN pip3 install scipy ipykernel warp-lang
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
RUN pip3 install matplotlib opencv-python proxsuite
RUN pip3 install isort black
RUN pip3 install warp-lang scikit-learn casadi mujoco pin

RUN pip install jupyter ipykernel
RUN pip install cyclonedds pygame 

# Set environmental variables required for using ROS 
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /unitree_ros2/cyclonedds_ws/install/setup.bash' >> ~/.bashrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
RUN echo 'export CYCLONEDDS_URI=file:///home/Go2py/Go2Py/assets/cyclonedds.xml' >> ~/.bashrc

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute