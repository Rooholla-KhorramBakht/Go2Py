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

    def reset(self):
        self.q_nominal = np.array(
            12*[0.]
        )
        for i in range(12):
            self.data.qpos[i] = self.q_nominal[i]

        self.data.qpos[7] = 0.0
        self.data.qpos[8] = 0.0

    def step(self, tau):
        self.step_counter += 1
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        if self.render and (self.step_counter%self.render_ds_ratio)==0:
            self.viewer.sync()

        return self.data.qpos, self.data.qvel


    def close(self):
        self.viewer.close()
