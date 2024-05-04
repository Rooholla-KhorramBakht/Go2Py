#!/bin/bash
set -e

if ! command -v git-lfs &> /dev/null
then
    echo "Installing git-lfs"
    sudo apt-get update
    sudo apt-get install \
        git-lfs
    echo "=============================================="
    echo "git-lfs is installed. Please remove cloned go2-devcontainer and re-clone it."
    echo "=============================================="
    exit 1
else
    echo "git-lfs is already installed"
fi

# # If foxglove-studio is not installed, install it
# if ! command -v foxglove-studio &> /dev/null
# then
#     echo "Installing foxglove-studio"
#     sudo snap install foxglove-studio
# else
#     echo "foxglove-studio is already installed"
# fi

# If VSCode is not installed, install it
if ! command -v code &> /dev/null
then
    echo "Installing VSCode"
    sudo snap install code --classic
else
    echo "VSCode is already installed"
fi
