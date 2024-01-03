#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting IDL type generation...${NC}"
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
echo -e "${GREEN} Done with LCM type generation${NC}"