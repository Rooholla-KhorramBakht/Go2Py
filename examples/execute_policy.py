from Go2Py.robot.interface.ros2 import GO2Real, ros2_init, ROS2ExecutorManager
import time
ros2_init()


import numpy as np

for i in range(10000):
    q = np.zeros(12)
    dq = np.zeros(12)
    kp = np.zeros(12)
    kd = np.zeros(12)
    tau = np.zeros(12)
    tau[0] = -0.8
    robot.setCommands(q, dq, kp, kd, tau)
    time.sleep(0.01)

def skew_symm(_v):
    a, b, c = _v(0), _v(1), _v(2)
    return np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])

def quat2rot(_q):
    q0 = _q(3)
    qv = _q[0:3]
    return (2 * q0**2 - 1) * np.eye(3) + 2 * q0 + skew_symm(qv) + 2 * qv @ qv.T

class ExecutePolicy():
    def __init__(self):
        self.initClass()
    
    def initClass(self):
        self.robot = GO2Real(mode='lowlevel')
        self.ros2_exec_manager = ROS2ExecutorManager()
        self.ros2_exec_manager.add_node(self.robot)
        self.ros2_exec_manager.start()
    
    def getObservation(self):
        self.js = self.robot.getJointStates()
        self.imu = self.robot.getIMU()
        # Calculate projected gravity vector as a torch array
        quat = self.imu['quat']
        R = quat2rot(self.imu['quat'])

        # Calculate current dof_pos as torch array
        self.obs_buf = torch.cat((self.projected_gravity,
                                (self.dof_pos[:, :self.num_actuated_dof] - self.default_dof_pos[:,
                                                                            :self.num_actuated_dof]) * self.obs_scales.dof_pos,
                                self.dof_vel[:, :self.num_actuated_dof] * self.obs_scales.dof_vel,
                                self.actions
                                ), dim=-1)
