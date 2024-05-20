#!/bin/bash

set -e

# Install thirdparty libraries that CANNOT be installed using rosdep
# Perfer to use rosdep to install thirdparty libraries if possible

mkdir -p /workspace/thirdparty
cd /workspace/thirdparty

# export CC=clang
# export CXX=clang++
# apt-get update

# ============================================
# Install magic_enum v0.9.5
# For some reason, ros-humble-magic-enum doesn't work
cd /workspace/thirdparty
git clone https://github.com/Neargye/magic_enum.git
cd magic_enum && git checkout v0.9.5
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DMAGIC_ENUM_OPT_BUILD_TESTS=OFF \
    -DMAGIC_ENUM_OPT_BUILD_EXAMPLES=OFF \
    ..
make -j$(nproc)
make install
cd /workspace/thirdparty && rm -rf magic_enum

# ============================================
# Install Open3D v.0.18.0
# The libopen3d-dev package is too old (0.14.0).
# cd /workspace/thirdparty
# git clone https://github.com/isl-org/Open3D.git
# cd Open3D && git checkout v0.18.0
# apt-get install -y libc++-dev libc++abi-dev
# mkdir build && cd build
# cmake \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DBUILD_SHARED_LIBS=ON \
#     -DBUILD_EXAMPLES=OFF \
#     -DBUILD_PYTHON_MODULE=OFF \
#     ..
# make -j$(nproc)
# make install
# cd /workspace/thirdparty && rm -rf Open3D


# ============================================
# Install google-glog v.0.7.0
# Some packages depend on it and we can't use libgoogle-glog-dev due to
# https://github.com/isl-org/Open3D/discussions/6515
cd /workspace/thirdparty
git clone https://github.com/google/glog.git
cd glog && git checkout v0.7.0
cmake -S . -B build -G "Unix Makefiles"
cmake --build build
cmake --build build --target install
cd /workspace/thirdparty && rm -rf glog

# ============================================
# Install unitree_sdk2
cd /workspace/thirdparty
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
./install.sh
cd /workspace/thirdparty && rm -rf unitree_sdk2