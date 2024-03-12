#!/bin/bash


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting LCM type generation...${NC}"
# Clean
rm -r unitree_lowlevel
# Make
lcm-gen -xp *.lcm
echo -e "${GREEN} Done with LCM type generation${NC}"
