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
        
        self.q0 = np.array([-0.03479636,  1.26186061, -2.81310153,
                             0.03325212,  1.25883281, -2.78329301,
                            -0.34708387,  1.27193761, -2.8052032 ,  
                             0.32040933,  1.27148342, -2.81436563])
        self.pos0 = np.array([0., 0., 0.1])
        self.rot0 = np.array([1., 0., 0., 0.])
        self.reset()
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        self.nv = self.model.nv
        self.jacp = np.zeros((3, self.nv))
        self.jacr = np.zeros((3, self.nv))
        self.M = np.zeros((self.nv, self.nv))

    def reset(self):
        self.q_nominal = np.hstack(
            [self.pos0.squeeze(), self.rot0.squeeze(), self.q0.squeeze()]
        )
        self.data.qpos = self.q_nominal
        self.data.qvel = np.zeros(18)
    
    def standUp(self):
        self.q0 = np.array([ 0.00901526,  0.77832842, -1.56065452,
                            -0.00795561,  0.76754963, -1.56634164,
                            -0.05375515,  0.76681757, -1.53601146,  
                             0.06183922,  0.75422204, -1.53229916])
        self.pos0 = np.array([0., 0., 0.33])
        self.rot0 = np.array([1., 0., 0., 0.])
        self.reset()
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def sitDown(self):
        self.q0 = np.array([-0.03479636,  1.26186061, -2.81310153,
                             0.03325212,  1.25883281, -2.78329301,
                            -0.34708387,  1.27193761, -2.8052032 ,  
                             0.32040933,  1.27148342, -2.81436563])
        self.pos0 = np.array([0., 0., 0.1])
        self.rot0 = np.array([1., 0., 0., 0.])
        self.reset()
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def getJointStates(self):
        return self.data.qpos[7:], self.data.qvel[6:]

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
        q, dq = self.getJointStates()
        tau = np.diag(kp)@(q_des-q).reshape(12,1)+ \
              np.diag(kv)@(dq_des-dq).reshape(12,1)+tau_ff.reshape(12,1)
        self.data.ctrl[:] = tau.squeeze()
        
    def step(self):
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

    def close(self):
        self.viewer.close()
