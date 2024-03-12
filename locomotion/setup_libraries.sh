SOURCE_DIR=$PWD
INSTALL_DIR=$SOURCE_DIR/custom/install

# install eigen3.4
cd $SOURCE_DIR/src/third_party/eigen
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release 
make -j$(($(nproc) - 2))
make install


# install pinocchio without python and install locally
cd $SOURCE_DIR/src/third_party/pinocchio
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_INTERFACE=OFF -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR
make -j$(($(nproc) - 2))
make install

# install mujoco locally
cd $SOURCE_DIR/src/third_party/mujoco
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR
cmake --build . -j$(($(nproc) - 2))
make install

cd $SOURCE_DIR/src/third_party/iir
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR
make -j$(($(nproc) - 2))
make install

cd $SOURCE_DIR/src/third_party/cyclonedds
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR # -DENABLE_TYPE_DISCOVERY=ON -DENABLE_TOPIC_DISCOVERY=ON
cmake --build . 
cmake --build . --target install

cd $SOURCE_DIR/src/third_party/cyclonedds-cxx
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_PREFIX_PATH=$INSTALL_DIR
cmake --build . 
cmake --build . --target install

cd $SOURCE_DIR
# chmod +x compile.sh
# ./compile.sh
