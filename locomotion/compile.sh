mkdir custom/build
cd custom/build
cmake ../.. -DCMAKE_BUILD_TYPE=Release
make -j$(($(nproc) - 2))
make install
cd ../..
