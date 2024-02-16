source /opt/ros/humble/setup.bash
source /unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /robot_ws/install/setup.bash && ros2 launch /root/launch/robot.launch.py