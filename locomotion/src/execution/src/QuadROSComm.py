import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import threading

class JointData:
    def __init__(self):
        self.q = np.array([0.] * 12)
        self.qd = np.array([0.] * 12)
        self.tau = np.array([0.] * 12)
        self.kp = np.array([0.] * 12)
        self.kd = np.array([0.] * 12)

class SensorData(JointData):
    def __init__(self):
        self.quat = np.array([0., 0., 0., 1.])
        self.a_B = np.array([0., 0., 0.])
        self.w_B = np.array([0., 0., 0.])
        self.a_W = np.array([0., 0., 0.])
        self.w_W = np.array([0., 0., 0.])

class QuadROSComm(Node):
    def __init__(self, name):
        super().__init__(name)
        self.m_name = name
        self.m_dt = 0.001
        self.thread = None
        self.initClass()

    def setLoopRate(self, rate):
        self.m_loop_rate = rate
        self.m_dt = 1./rate
    
    def initClass(self):
        self.joint_cmd = JointState()
        # self.joint_cmd.position.clear()
        # self.joint_cmd.velocity.clear()
        # self.joint_cmd.effort.clear()
        self.joint_cmd.position = [0.] * 12
        self.joint_cmd.velocity = [0.] * 12
        self.joint_cmd.effort = [0.] * 12
        print("Joint cmd init: ", self.joint_cmd.position)

        self.joint_state = JointState()
        # self.joint_state.position.clear()
        # self.joint_state.velocity.clear()
        # self.joint_state.effort.clear()
        self.joint_state.position = [0.] * 12
        self.joint_state.velocity = [0.] * 12
        self.joint_state.effort = [0.] * 12

        self.imu = Imu()
        self.sensor_data = SensorData()
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/" + self.m_name + "/joint_state",
            self.get_joint_state_cb,
            1)
        self.joint_state_sub
        self.imu_sub = self.create_subscription(
            Imu,
            "/" + self.m_name + "/imu",
            self.get_imu_cb,
            1)
        self.imu_sub
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            "/" + self.m_name + "/joint_cmd",
            1)
        self.timer = self.create_timer(self.m_dt, self.timer_callback)
    
    def set_joint_command(self, jcmd):
        # print("Joint position current: ", self.joint_cmd.position)
        # print("Joint position command: ", jcmd.q)
        self.joint_cmd.position = list(jcmd.q)
        self.joint_cmd.velocity = list(jcmd.qd)
        self.joint_cmd.effort = list(jcmd.tau)
    
    def timer_callback(self):
        self.joint_cmd_pub.publish(self.joint_cmd)
    
    def get_joint_state_cb(self, msg):
        self.joint_state = msg

    def get_imu_cb(self, msg):
        self.imu = msg

    def getSensorData(self):
        self.sensor_data.q = np.array(self.joint_state.position)
        self.sensor_data.qd = np.array(self.joint_state.velocity)
        self.sensor_data.tau = np.array(self.joint_state.effort)
        self.sensor_data.quat[0] = self.imu.orientation.x
        self.sensor_data.quat[1] = self.imu.orientation.y
        self.sensor_data.quat[2] = self.imu.orientation.z
        self.sensor_data.quat[3] = self.imu.orientation.w
        # TODO: update the sensor data object as required. Currently only this much is needed to get observations for policy
        return self.sensor_data
    
    def getJointData(self):
        return {'q': self.joint_state.position, 'qd': self.joint_state.velocity}
    def getImuData(self):
        return {}
    
    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        self.thread.join()

    def start_thread(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()