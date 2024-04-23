docker run --rm -it --privileged --network host -v $(pwd)/deploy/nav2_ws/src:/home/nav2_ws/src -v /dev/*:/dev/* -v /etc/localtime:/etc/localtime:ro --runtime nvidia go2py_nav:latest
