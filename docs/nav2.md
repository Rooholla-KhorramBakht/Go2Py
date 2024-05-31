# Nav2 with Go2
This notes describe the process getting an exploration and mapping sytem running on the robot using the Go2Py docker and configuration files. In this setup we use the following components:
- **ROS Robot Localization:** We use the EKF node of the ROS robot localization package to generate the odometry TF transformations based on the robot's buildin leg inertial odometry systm (`/utlidar/robot_odom`). The go2py_bridge listens to this topic and publishes `/go2/odom` messages re-stamped with dock's internal clock. The configuraiton parameters for this node are [here](). 

- **ROS SLAM Toolbox:** FOR SLAM, we use the ROS SLAM toobox that takes the odometry frames from the localization node alongside the laser scan topics from the hesai LiDAR (`/go2/scan`) to make a 2D occupancy map of the environment and the transformation between the `odom` and `map` frame. 

- **ROS2 Navigation:** Finally we use the ROS navigation2 to 
