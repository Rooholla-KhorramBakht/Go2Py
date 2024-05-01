from Go2Py.robot.interface.ros2 import GO2Real, ros2_init, ROS2ExecutorManager
import time
ros2_init()
robot = GO2Real(mode='highlevel')
