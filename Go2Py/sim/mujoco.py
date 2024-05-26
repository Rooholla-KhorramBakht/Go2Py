import time
import mujoco
import mujoco.viewer
import numpy as np
from Go2Py import ASSETS_PATH
import os
from scipy.spatial.transform import Rotation

pnt = np.array([-0.2, 0, 0.05])
lidar_angles = np.linspace(0.0, 2 * np.pi, 1024).reshape(-1, 1)
x_vec = np.cos(lidar_angles)
y_vec = np.sin(lidar_angles)
z_vec = np.zeros_like(x_vec)
vec = np.concatenate([x_vec, y_vec, z_vec], axis=1)
nray = vec.shape[0]
geomid = np.zeros(nray, np.int32)
dist = np.zeros(nray, np.float64)


class Go2Sim:
    def __init__(self, mode='lowlevel', render=True, dt=0.002, xml_path=None):

        if xml_path is None:
            self.model = mujoco.MjModel.from_xml_path(
                os.path.join(ASSETS_PATH, 'mujoco/go2.xml')
            )
        else:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.simulated = True
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        _render_dt = 1 / 60
        self.render_ds_ratio = max(1, _render_dt // dt)

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

        self.prestanding_q = np.array([0.0, 1.26186061, -2.5,
                                       0.0, 1.25883281, -2.5,
                                       0.0, 1.27193761, -2.6,
                                       0.0, 1.27148342, -2.6])

        self.sitting_q = np.array([-0.02495611, 1.26249647, -2.82826662,
                                   0.04563564, 1.2505368, -2.7933557,
                                   -0.30623949, 1.28283751, -2.82314873,
                                   0.26400229, 1.29355574, -2.84276843])

        self.standing_q = np.array([0.0, 0.77832842, -1.56065452,
                                    0.0, 0.76754963, -1.56634164,
                                    0.0, 0.76681757, -1.53601146,
                                    0.0, 0.75422204, -1.53229916])

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
        self.actuator_tau = np.zeros(12)
        self.mode = mode
        if self.mode == 'highlevel':
            from Go2Py.control.walk_these_ways import CommandInterface, loadParameters, Policy, WalkTheseWaysAgent, HistoryWrapper
            checkpoint_path = os.path.join(ASSETS_PATH,'checkpoints/walk_these_ways')
            self.cfg = loadParameters(checkpoint_path)
            self.policy = Policy(checkpoint_path)
            self.command_profile = CommandInterface()
            self.agent = WalkTheseWaysAgent(self.cfg, self.command_profile, robot=self)
            self.agent = HistoryWrapper(self.agent)
            self.control_dt = self.cfg["control"]["decimation"] * self.cfg["sim"]["dt"]
            self.obs = self.agent.reset()
            self.standUpReset()
            self.step_counter = 0
            self.step = self.stepHighlevel
        else:
            self.step = self.stepLowlevel

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
        return {"q": self.data.qpos[7:],
                "dq": self.data.qvel[6:],
                'tau_est': self.actuator_tau}

    def getPose(self):
        return self.data.qpos[:3], self.data.qpos[3:7]

    def getIMU(self):
        return {
            'accel': np.array(self.data.sensordata[0:3]),
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

    def stepLowlevel(self):
        state = self.getJointStates()
        q, dq = state['q'], state['dq']
        tau = np.diag(self.kp) @ (self.q_des - q).reshape(12, 1) + \
            np.diag(self.kv) @ (self.dq_des - dq).reshape(12, 1) + self.tau_ff.reshape(12, 1)
        self.actuator_tau = tau
        self.data.ctrl[:] = tau.squeeze()

        self.step_counter += 1
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        if self.render and (self.step_counter % self.render_ds_ratio) == 0:
            self.viewer.sync()

    def stepHighlevel(self, vx, vy, omega_z, body_z_offset=0):
        policy_info = {}
        if self.step_counter % (self.control_dt // self.dt) == 0:
            action = self.policy(self.obs, policy_info)
            self.obs, ret, done, info = self.agent.step(action)
            
        self.step_counter+=1
        self.stepLowlevel()
        self.command_profile.yaw_vel_cmd = omega_z
        self.command_profile.x_vel_cmd = vx
        self.command_profile.y_vel_cmd = vy  
        self.command_profile.body_height_cmd = body_z_offset

    def getSiteJacobian(self, site_name):
        id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        assert id > 0, 'The requested site could not be found'
        mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, id)
        return self.jacp, self.jacr

    def getDynamicsParams(self):
        mujoco.mj_fullM(self.model, self.M, self.data.qM)
        nle = self.data.qfrc_bias.reshape(self.nv, 1)
        return {
            'M': self.M,
            'nle': nle
        }

    def getGravityInBody(self):
        _, q = self.getPose()
        R = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        g_in_body = R.T @ np.array([0.0, 0.0, -1.0]).reshape(3, 1)
        return g_in_body

    def getLidarData(self):
        t, q = self.getPose()
        world_R_body = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        pnt = t.copy()
        pnt[2]+=0.25
        vec_in_w = (world_R_body@vec.T).T
        mujoco.mj_multiRay(
            m=self.model,
            d=self.data,
            pnt=pnt,
            vec=vec.flatten(),
            geomgroup=None,
            flg_static=1,
            bodyexclude=-1,
            geomid=geomid,
            dist=dist,
            nray=nray,
            cutoff=mujoco.mjMAXVAL,
        )
        pcd = dist.reshape(-1, 1) * vec
        return {"pcd": pcd, "geomid": geomid, "dist": dist}

    def overheat(self):
        return False

    def close(self):
        self.viewer.close()
