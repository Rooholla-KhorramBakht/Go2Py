#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting DDS type generation...${NC}"
# Clean
rm -r msgs
# Make
for file in idl/*.idl
do
    echo "Processing $file file..."
    idlc -l py $file
done
mkdir msgs/cpp
cd msgs/cpp
for file in ../../idl/*.idl
do
    echo "Processing $file file..."
    idlc -l ../../idl/libcycloneddsidlcxx.so.0.10.2 $file
done
cd ../..
# rm -r ../cpp_bridge/include/go2py
# mv msgs/cpp ../cpp_bridge/include/go2py
echo -e "${GREEN} Done with DDS type generation${NC}"