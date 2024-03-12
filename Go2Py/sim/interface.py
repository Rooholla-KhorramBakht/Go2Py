import time

import numpy as np

from B1Py import B1_ISAACSIM_CFG_PATH
from B1Py.lcm_types.unitree_lowlevel import UnitreeLowCommand, UnitreeLowState
from B1Py.sim.utils import LCMBridgeClient, NumpyMemMapDataPipe, load_config


class B1IsaacSim:
    """
    Class for communication with the simulated Franka Emika FR3 robot in Isaac Sim.
    It uses LCM to send joint velocity to the simulated robot and receive joint states
    (joint angle, velocity, and torque) and camera images from the robot.
    """

    def __init__(self, robot_id="b1"):
        """
        @param robot_id: (str) The name of the robot in the Isaac Sim scene.
        """
        self.lcm_bridge = LCMBridgeClient(robot_name=robot_id)
        self.cfg = load_config(B1_ISAACSIM_CFG_PATH)
        self.camera_pipes = {}
        for camera in self.cfg["cameras"]:
            for type in camera["type"]:
                if type == "rgb":
                    shape = (camera["resolution"][1], camera["resolution"][0], 4)
                else:
                    shape = (camera["resolution"][1], camera["resolution"][0])

                self.camera_pipes[camera["name"] + "_" + type] = NumpyMemMapDataPipe(
                    camera["name"] + "_" + type, force=False, shape=shape
                )
            print(camera["name"] + "_" + type)

    def LCMThreadFunc(self):
        """
        Function that runs on a separate thread and handles LCM communication.
        `lc.handle()` function is called when there are data available to be processed.
        """
        while self.running:
            rfds, wfds, efds = select.select([self.lc.fileno()], [], [], 0.5)
            if rfds:  # Handle only if there are data in the interface file
                self.lc.handle()

    def getStates(self):
        """
        Get the current joint angle, velocity, and torque of the robot.

        @return: (dict or None) The joint state {'q': (numpy.ndarray, shape=(n,)),
        'dq': (numpy.ndarray, shape=(n,)), 'T': (numpy.ndarray, shape=(n,))}
        if the latest update was received less than 0.2 seconds ago;
        otherwise, return None.
        """
        state = self.lcm_bridge.getStates(timeout=0.1)
        if state is not None:
            q = np.array(state.q)
            dq = np.array(state.dq)
            tau_est = np.array(state.tau_est)
            contact_state = np.array(state.contact_state)
            accel = np.array(state.accel)
            gyro = np.array(state.gyro)
            temperature = np.array(state.temperature)
            quaternion = np.array(state.quaternion)
            rpy = np.array(state.rpy)
            gravity = np.array(state.gravity)
            gt_pos = np.array(state.gt_pos)
            gt_quat = np.array(state.gt_quat)
            gt_lin_vel = np.array(state.gt_lin_vel)
            gt_ang_vel = np.array(state.gt_ang_vel)
            return {
                "q": q,
                "dq": dq,
                "tau_est": tau_est,
                "contact_state": contact_state,
                "accel": accel,
                "gyro": gyro,
                "temperature": temperature,
                "quaternion": quaternion,
                "rpy": rpy,
                "gravity": gravity,
                "gt_pos": gt_pos,
                "gt_quat": gt_quat,
                "gt_lin_vel": gt_lin_vel,
                "gt_ang_vel": gt_ang_vel,
            }

        else:
            return None

    def readCameras(self):
        data = {key: pipe.read() for key, pipe in self.camera_pipes.items()}
        return data

    def sendCommands(self, kp, kd, q_des, dq_des, tau_ff):
        """
        Send a joint velocity command to the robot.
        @param kp: (numpy.ndarray, shape=(n,)) The proportional gain for the joint position control.
        @param kd: (numpy.ndarray, shape=(n,)) The derivative gain for the joint position control.
        @param q_des: (numpy.ndarray, shape=(n,)) The desired joint position.
        @param dq_des: (numpy.ndarray, shape=(n,)) The desired joint velocity.
        @param tau_ff: (numpy.ndarray, shape=(n,)) The feedforward torque.
        """
        cmd_lcm = UnitreeLowCommand()
        cmd_lcm.tau_ff = tau_ff.tolist()
        cmd_lcm.kp = kp.tolist()
        cmd_lcm.kd = kd.tolist()
        cmd_lcm.q_des = q_des.tolist()
        cmd_lcm.dq_des = dq_des.tolist()
        self.lcm_bridge.sendCommands(cmd_lcm)

    def close(self):
        """
        Stop the LCM thread, unsubscribe from the LCM topic,
        and effectively shut down the interface.
        """
        self.lcm_bridge.close()

    def reset(self):
        time.sleep(0.3)
        for i in range(100):
            time.sleep(0.01)
            self.sendCommands(np.zeros(9))
            state = self.getStates()
