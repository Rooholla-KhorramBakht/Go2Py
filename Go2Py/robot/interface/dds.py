import struct
import threading
import time
import numpy as np
import numpy.linalg as LA
from scipy.spatial.transform import Rotation as R

from cyclonedds.domain import DomainParticipant
from Go2Py.unitree_go.msg.dds_ import Go2pyLowCmd_
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.util import duration
from Go2Py.unitree_go.msg.dds_ import LowState_
from threading import Thread

class GO2Real():
    def __init__(
        self,
        mode = 'lowlevel', # 'highlevel' or 'lowlevel'
        vx_max=0.5,
        vy_max=0.4,
        ωz_max=0.5,
    ):
        assert mode in ['highlevel', 'lowlevel'], "mode should be either 'highlevel' or 'lowlevel'"
        self.mode = mode
        if self.mode == 'highlevel':
            raise NotImplementedError('DDS interface for the highlevel commands is not implemented yet. Please use our ROS2 interface.')
        self.simulated = False
        self.highcmd_topic_name = "rt/go2/twist_cmd"
        self.lowcmd_topic_name = "rt/go2/lowcmd"
        self.lowstate_topic_name = "rt/lowstate"

        self.participant = DomainParticipant()
        self.lowstate_topic = Topic(self.participant, self.lowstate_topic_name, LowState_)
        self.lowstate_reader = DataReader(self.participant, self.lowstate_topic)
        
        self.lowcmd_topic = Topic(self.participant, self.lowcmd_topic_name, Go2pyLowCmd_)
        self.lowcmd_writer = DataWriter(self.participant, self.lowcmd_topic)

        self.state = None
        self.setCommands = {'lowlevel':self.setCommandsLow}[self.mode]
        self.lowstate_thread = Thread(target = self.lowstate_update)
        self.running = True
        self.lowstate_thread.start()

    def lowstate_update(self):
        """
        Retrieve the state of the robot
        """
        while self.running:
            for msg in self.lowstate_reader.take_iter(timeout=duration(milliseconds=100.)):
                self.state = msg

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
        motor_state = np.array([[self.state.motor_state[i].q,
                                 self.state.motor_state[i].dq,
                                 self.state.motor_state[i].ddq,
                                 self.state.motor_state[i].tau_est,
                                 self.state.motor_state[i].temperature] for i in range(12)])
        return {'q':motor_state[:,0], 
                'dq':motor_state[:,1],
                'ddq':motor_state[:,2],
                'tau_est':motor_state[:,3],
                'temperature':motor_state[:,4]}

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
        batteryState = self.state.bms_state
        return batteryState.soc

    def setCommandsHigh(self, v_x, v_y, ω_z, bodyHeight=0.0, footRaiseHeight=0.0, mode=2):
        self.cmd_watchdog_timer = time.time()
        _v_x, _v_y, _ω_z = self.clip_velocity(v_x, v_y, ω_z)
        self.highcmd.header.stamp = self.get_clock().now().to_msg()
        self.highcmd.header.frame_id = "base_link"
        self.highcmd.twist.linear.x = _v_x
        self.highcmd.twist.linear.y = _v_y
        self.highcmd.twist.angular.z = _ω_z
        self.highcmd_publisher.publish(self.highcmd)

    def setCommandsLow(self, q_des, dq_des, kp, kd, tau_ff):
        assert q.size == dq.size == kp.size == kd.size == tau_ff.size == 12, "q, dq, kp, kd, tau_ff should have size 12"
        lowcmd = Go2pyLowCmd_(
            q,
            dq, 
            kp,
            kd,
            tau_ff
        )
        self.lowcmd_writer.write(lowcmd)

    def close(self):
        self.running = False

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