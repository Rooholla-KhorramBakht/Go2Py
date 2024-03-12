
mkdir custom/build
cd custom/build
cmake ../.. -DCMAKE_BUILD_TYPE=Release -DINSTALL_HEADERS_ONLY=true
 make install -j$(($(nproc) - 2))
cmake ../.. -DCMAKE_BUILD_TYPE=Release -DINSTALL_HEADERS_ONLY=false
make install -j$(($(nproc) - 2))
# make install
cd ../..
