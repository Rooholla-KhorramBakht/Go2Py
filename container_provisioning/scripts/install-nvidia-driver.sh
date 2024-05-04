#!/bin/bash
set -e

if ! command -v nvidia-smi &> /dev/null
then
    echo "Installing NVIDIA driver"
else
    echo "NVIDIA driver is already installed"
    exit 0
fi

# Prompt user to confirm to install NVIDIA driver 535
NVIDIA_DRIVER_VERSION=535
while true; do
    read -p "Do you want to install NVIDIA driver ${NVIDIA_DRIVER_VERSION}? (y/n)" yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) echo "Please install NVIDIA driver manually!"; exit 1;;
        * ) echo "Please answer yes or no.";;
    esac
done

sudo apt-get update
sudo apt-get install -y nvidia-driver-${NVIDIA_DRIVER_VERSION}