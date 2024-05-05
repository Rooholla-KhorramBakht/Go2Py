#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting DDS type generation...${NC}"
# Clean
rm -r ../unitree_go
rm -r ../go2py_common

# Make
for file in idls/*.idl
do
    echo "Processing $file file..."
    idlc -l py $file
done
mv unitree_go ../
mv go2py_common ../