#!/bin/bash
set -e

if ! [ -x "$(command -v curl)" ]; then
    sudo apt-get install curl
fi

# If docker is not installed, install it
if ! [ -x "$(command -v docker)" ]; then
    echo "Docker is not installed. Installing Docker"
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh

    # Post-installation steps: https://docs.docker.com/engine/install/linux-postinstall/
    if ! [ $(getent group docker) ]; then
        sudo groupadd docker
    fi
    sudo usermod -aG docker $USER

    echo "=============================================="
    echo "Docker installed. Please restart your computer."
    echo "=============================================="
    exit 1
fi

# Check if docker is running
if ! systemctl is-active --quiet docker; then
    echo "Docker is not running. Please start Docker and run this script again."
    exit 1
fi

# If nvidia-container-toolkit is not installed, install it
# Follow https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt
if ! [ -x "$(command -v nvidia-container-toolkit)" ]; then
    # Install nvidia-container-toolkit
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    # Configure nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    nvidia-ctk runtime configure --runtime=docker --config=$HOME/.config/docker/daemon.json
    systemctl --user restart docker
    sudo nvidia-ctk config --set nvidia-container-cli.no-cgroups --in-place
fi