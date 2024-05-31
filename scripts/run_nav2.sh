#!/bin/bash

CONTAINER_NAME="go2py_nav2"

# Function to print usage
function usage() {
    echo "Usage: run_go2py_nav2.sh"
}

# Function to check if docker command succeeded
function check_docker() {
    if [ $? -ne 0 ]; then
        echo "Docker command failed. Please check your Docker installation."
        exit 1
    fi
}

# Prevent running as root
if [ $(id -u) -eq 0 ]; then
    echo "This script cannot be executed with root privileges."
    echo "Please re-run without sudo and configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root
if [[ ! $(groups $USER) =~ docker ]]; then
    echo "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    echo "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    echo "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands
docker ps &>/dev/null
check_docker

# Check if the container is running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Attaching to running container: $CONTAINER_NAME"
    docker exec -it --workdir /home/nav2_ws $CONTAINER_NAME /bin/bash 
    check_docker
else
    echo "Starting new container: $CONTAINER_NAME"
    sudo docker run -it --rm --name $CONTAINER_NAME --privileged --network host -v $(pwd)/deploy/ros2_nodes/sportmode_nav2:/home/nav2_ws/src/sportmode_nav2 -v /dev/*:/dev/* -v /etc/localtime:/etc/localtime:ro --runtime nvidia --workdir /home/nav2_ws go2py_nav2:latest  
    check_docker
fi
