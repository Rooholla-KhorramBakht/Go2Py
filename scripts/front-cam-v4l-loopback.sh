#!/bin/bash

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if v4l2loopback is installed and install it if not
if ! command_exists modprobe; then
    echo "modprobe command not found. Please install kmod package."
    exit 1
fi

if ! lsmod | grep -q v4l2loopback; then
    echo "v4l2loopback module not loaded. Checking installation..."
    if ! dpkg -s v4l2loopback-dkms >/dev/null 2>&1; then
        echo "v4l2loopback not installed. Installing..."
        sudo apt-get update
        sudo apt-get install -y v4l2loopback-dkms v4l2loopback-utils
    fi
    echo "Loading v4l2loopback module..."
    sudo modprobe v4l2loopback
else
    echo "v4l2loopback module already loaded."
fi

# Run the GStreamer pipeline
gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! queue ! application/x-rtp, media=video, \
encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! v4l2sink device=/dev/video0
