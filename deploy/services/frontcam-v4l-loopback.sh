/usr/sbin/modprobe v4l2loopback
# Run the GStreamer pipeline
/usr/bin/gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! queue ! application/x-rtp, media=video, \
encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! v4l2sink device=/dev/video6 &
sleep 1
# Run the ROS image publisher node
/usr/bin/docker run --rm --name go2py_frontcam --privileged --network host -v /dev/*:/dev/* -v /etc/localtime:/etc/localtime:ro go2py_frontcam_publisher:latest
