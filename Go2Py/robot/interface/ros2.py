import struct
import threading
import time
import numpy as np
import numpy.linalg as LA
from scipy.spatial.transform import Rotation as R
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped
from Go2Py.joy import xKeySwitch, xRockerBtn
from geometry_msgs.msg import TwistStamped
from Go2Py.msgs.unitree_go.msg import LowState, Go2pyLowCmd
from nav_msgs.msg import Odometry   



def ros2_init(args=None):
    rclpy.init(args=args)


def ros2_close():
    rclpy.shutdown()

class ROS2ExecutorManager:
    """A class to manage the ROS2 executor. It allows to add nodes and start the executor in a separate thread."""
    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self.nodes = []
        self.executor_thread = None

    def add_node(self, node: Node):
        """Add a new node to the executor."""
        self.nodes.append(node)
        self.executor.add_node(node)

    def _run_executor(self):
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.terminate()

    def start(self):
        """Start spinning the nodes in a separate thread."""
        self.executor_thread = threading.Thread(target=self._run_executor)
        self.executor_thread.start()

    def terminate(self):
        """Terminate all nodes and shutdown rclpy."""
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()
        if self.executor_thread:
            self.executor_thread.join()

class GO2Real(Node):
    def __init__(
        self,
        mode = 'highlevel', # 'highlevel' or 'lowlevel'
        vx_max=0.5,
        vy_max=0.4,
        ωz_max=0.5,
    ):
        assert mode in ['highlevel', 'lowlevel'], "mode should be either 'highlevel' or 'lowlevel'"
        self.simulated = False
        self.mode = mode
        self.node_name = "go2py_highlevel_subscriber"
        self.highcmd_topic = "/go2/twist_cmd"
        self.lowcmd_topic = "/go2/lowcmd"
        self.joint_state_topic = "/go2/joint_states"
        self.lowstate_topic = "/lowstate"
        super().__init__(self.node_name)
        
        self.lowstate_subscriber = self.create_subscription(
            LowState, self.lowstate_topic, self.lowstate_callback, 1
        )
        self.lowcmd_publisher = self.create_publisher(Go2pyLowCmd, self.lowcmd_topic, 1)

        self.odometry_subscriber = self.create_subscription(
            Odometry, "/utlidar/robot_odom", self.odom_callback, 1
        )

        self.highcmd_publisher = self.create_publisher(TwistStamped, self.highcmd_topic, 1)
        self.highcmd = TwistStamped()
        # create pinocchio robot
        # self.pin_robot = PinRobot()

        # for velocity clipping
        self.vx_max = vx_max
        self.vy_max = vy_max
        self.P_v_max = np.diag([1 / self.vx_max**2, 1 / self.vy_max**2])
        self.ωz_max = ωz_max
        self.ωz_min = -ωz_max
        self.running = True
        self.setCommands = {'lowlevel':self.setCommandsLow,
                            'highlevel':self.setCommandsHigh}[self.mode]

    def lowstate_callback(self, msg):
        """
        Retrieve the state of the robot
        """
        self.state = msg

    def odom_callback(self, msg):
        """
        Retrieve the odometry of the robot
        """
        self.odom = msg

    def getOdometry(self):
        """Returns the odometry of the robot"""
        stamp = self.odom.header.stamp
        position = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
        orientation = np.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
        stamp_nanosec = stamp.sec + stamp.nanosec * 1e-9
        return {'stamp_nanosec':stamp_nanosec, 'position':position, 'orientation':orientation}

    def getIMU(self):
        accel = self.state.imu_state.accelerometer
        gyro = self.state.imu_state.gyroscope
        quat = self.state.imu_state.quaternion
        rpy = self.state.imu_state.rpy
        temp = self.state.imu_state.temperature
        return {'accel':accel, 'gyro':gyro, 'quat':quat, "rpy":rpy, 'temp':temp}

    def getFootContacts(self):
        """Returns the raw foot contact forces"""
        footContacts = self.state.foot_force
        return np.array(footContacts)

    def getJointStates(self):
        """Returns the joint angles (q) and velocities (dq) of the robot"""
        motorStates = self.state.motor_state
        _q, _dq = zip(
            *[(motorState.q, motorState.dq) for motorState in motorStates[:12]]
        )
        q, dq = np.array(_q), np.array(_dq)

        return {'q':q, 'dq':dq}

    def getRemoteState(self):
        """A method to get the state of the wireless remote control. 
        Returns a xRockerBtn object: 
        - head: [head1, head2]
        - keySwitch: xKeySwitch object
        - lx: float
        - rx: float
        - ry: float
        - L2: float
        - ly: float
        """
        wirelessRemote = self.state.wireless_remote[:24]

        binary_data = bytes(wirelessRemote)

        format_str = "<2BH5f"
        data = struct.unpack(format_str, binary_data)

        head = list(data[:2])
        lx = data[3]
        rx = data[4]
        ry = data[5]
        L2 = data[6]
        ly = data[7]

        _btn = bin(data[2])[2:].zfill(16)
        btn = [int(char) for char in _btn]
        btn.reverse()

        keySwitch = xKeySwitch(*btn)
        rockerBtn = xRockerBtn(head, keySwitch, lx, rx, ry, L2, ly)
        return rockerBtn

    def getCommandFromRemote(self):
        """Do not use directly for control!!!"""
        rockerBtn = self.getRemoteState()

        lx = rockerBtn.lx
        ly = rockerBtn.ly
        rx = rockerBtn.rx

        v_x = ly * self.vx_max
        v_y = lx * self.vy_max
        ω = rx * self.ωz_max
        
        return v_x, v_y, ω

    def getBatteryState(self):
        """Returns the battery percentage of the robot"""
        batteryState = self.state.bms
        return batteryState.SOC

    def setCommandsHigh(self, v_x, v_y, ω_z, bodyHeight=0.0, footRaiseHeight=0.0, mode=2):
        self.cmd_watchdog_timer = time.time()
        _v_x, _v_y, _ω_z = self.clip_velocity(v_x, v_y, ω_z)
        self.highcmd.header.stamp = self.get_clock().now().to_msg()
        self.highcmd.header.frame_id = "base_link"
        self.highcmd.twist.linear.x = _v_x
        self.highcmd.twist.linear.y = _v_y
        self.highcmd.twist.angular.z = _ω_z
        self.highcmd_publisher.publish(self.highcmd)

    def setCommandsLow(self, q, dq, kp, kd, tau_ff):
        assert q.size == dq.size == kp.size == kd.size == tau_ff.size == 12, "q, dq, kp, kd, tau_ff should have size 12"
        lowcmd = Go2pyLowCmd()
        lowcmd.q = q.tolist()
        lowcmd.dq = dq.tolist()
        lowcmd.kp = kp.tolist()
        lowcmd.kd = kd.tolist()
        lowcmd.tau = tau_ff.tolist()
        self.lowcmd_publisher.publish(lowcmd)

    def close(self):
        self.running = False
        self.thread.join()
        self.destroy_node()

    def check_calf_collision(self, q):
        self.pin_robot.update(q)
        in_collision = self.pin_robot.check_calf_collision(q)
        return in_collision

    def clip_velocity(self, v_x, v_y, ω_z):
        _v = np.array([[v_x], [v_y]])
        _scale = np.sqrt(_v.T @ self.P_v_max @ _v)[0, 0]

        if _scale > 1.0:
            scale = 1.0 / _scale
        else:
            scale = 1.0

        return scale * v_x, scale * v_y, np.clip(ω_z, self.ωz_min, self.ωz_max)