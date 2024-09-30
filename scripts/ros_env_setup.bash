source /opt/ros/humble/setup.bash
source /unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/Go2Py/assets/cyclonedds.xml