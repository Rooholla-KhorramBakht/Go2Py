import time
from copy import deepcopy
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from Go2Py import ASSETS_PATH
import os

class Go2Sim:
    def __init__(self, render=True, dt=0.002):
        
        self.model = mujoco.MjModel.from_xml_path(
            os.path.join(ASSETS_PATH, 'mujoco/go2.xml')
        )
        self.simulated = True
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        _render_dt = 1/60
        self.render_ds_ratio = max(1, _render_dt//dt)

        if render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.render = True
            self.viewer.cam.distance = 3.0
            self.viewer.cam.azimuth = 90
            self.viewer.cam.elevation = -45
            self.viewer.cam.lookat[:] = np.array([0.0, -0.25, 0.824])
        else:
            self.render = False

        self.model.opt.gravity[2] = -9.81
        self.model.opt.timestep = dt
        self.renderer = None
        self.render = render
        self.step_counter = 0
        
        self.prestanding_q = np.array([ 0.0,  1.26186061, -2.5,
                                    0.0,  1.25883281, -2.5,
                                    0.0,  1.27193761, -2.6,  
                                    0.0,  1.27148342, -2.6])

        self.sitting_q = np.array([-0.02495611,  1.26249647, -2.82826662,
                                    0.04563564,  1.2505368 , -2.7933557 ,
                                   -0.30623949,  1.28283751, -2.82314873,  
                                    0.26400229,  1.29355574, -2.84276843])

        self.standing_q = np.array([ 0.0,  0.77832842, -1.56065452,
                                     0.0,  0.76754963, -1.56634164,
                                     0.0,  0.76681757, -1.53601146,  
                                     0.0,  0.75422204, -1.53229916])

        self.q0 = self.sitting_q
        self.pos0 = np.array([0., 0., 0.1])
        self.rot0 = np.array([1., 0., 0., 0.])
        self.reset()
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        self.nv = self.model.nv
        self.jacp = np.zeros((3, self.nv))
        self.jacr = np.zeros((3, self.nv))
        self.M = np.zeros((self.nv, self.nv))

        self.q_des = np.zeros(12)
        self.dq_des = np.zeros(12)
        self.tau_ff = np.zeros(12)
        self.kp = np.zeros(12)
        self.kv = np.zeros(12)
        self.latest_command_stamp = time.time()

    def reset(self):
        self.q_nominal = np.hstack(
            [self.pos0.squeeze(), self.rot0.squeeze(), self.q0.squeeze()]
        )
        self.data.qpos = self.q_nominal
        self.data.qvel = np.zeros(18)
    
    def standUpReset(self):
        self.q0 = self.standing_q
        self.pos0 = np.array([0., 0., 0.33])
        self.rot0 = np.array([1., 0., 0., 0.])
        self.reset()
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def sitDownReset(self):
        self.q0 = self.sitting_q
        self.pos0 = np.array([0., 0., 0.1])
        self.rot0 = np.array([1., 0., 0., 0.])
        self.reset()
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def getJointStates(self):
        return {"q":self.data.qpos[7:], 
               "dq":self.data.qvel[6:]}

    def getPose(self):
        return self.data.qpos[:3], self.data.qpos[3:7]

    def getIMU(self):
        return {
                'accel':np.array(self.data.sensordata[0:3]),\
                'gyro': np.array(self.data.sensordata[3:6])
               }

    def getFootContact(self):
        return self.data.sensordata[6:10]

    def setCommands(self, q_des, dq_des, kp, kv, tau_ff):
        self.q_des = q_des
        self.dq_des = dq_des
        self.kp = kp
        self.kv = kv
        self.tau_ff = tau_ff
        self.latest_command_stamp = time.time()
        
    def step(self):
        state = self.getJointStates()
        q, dq = state['q'], state['dq']
        tau = np.diag(self.kp)@(self.q_des-q).reshape(12,1)+ \
              np.diag(self.kv)@(self.dq_des-dq).reshape(12,1)+self.tau_ff.reshape(12,1)
        self.data.ctrl[:] = tau.squeeze()

        self.step_counter += 1
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        if self.render and (self.step_counter%self.render_ds_ratio)==0:
            self.viewer.sync()

    def getSiteJacobian(self, site_name):
        id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE,site_name)
        assert id>0, 'The requested site could not be found'
        mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, id)
        return self.jacp, self.jacr

    def getDynamicsParams(self):
        mujoco.mj_fullM(self.model, self.M, self.data.qM)
        nle = self.data.qfrc_bias.reshape(self.nv,1)
        return {
            'M':self.M,
            'nle':nle
        }

    def overheat(self):
        return False

    def close(self):
        self.viewer.close()
