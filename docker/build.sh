#!/bin/bash
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
    DOCKERFILE_PATH="docker/Dockerfile.x86"
elif [ "$ARCH" = "aarch64" ]; then
    DOCKERFILE_PATH="docker/Dockerfile.arm64"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

docker build --file $DOCKERFILE_PATH --tag go2py-container .
