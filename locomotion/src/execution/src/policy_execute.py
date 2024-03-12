import torch
import numpy as np
import pickle as pk
import os
from QuadROSComm import *
import time

class Policy:
    def __init__(self, path):
        self.body = torch.jit.load(os.path.join(path, 'checkpoints/body_latest.jit'))
        self.adaptation_module = torch.jit.load(os.path.join(path,'checkpoints/adaptation_module_latest.jit'))
    
    def forward(self, obs_hist, info={}):
        latent = self.adaptation_module.forward(obs_hist.to('cpu'))
        action = self.body.forward(torch.cat((obs_hist.to('cpu'), latent), dim=-1))
        info['latent'] = latent
        return action

class TorchBuffer:
        def __init__(self, vector_size, history_size):
            self.device = "cpu"
            self.vector_size = vector_size
            self.history_size = history_size
            self.buffer_size = self.vector_size * self.history_size
            self.buffer = torch.zeros(self.buffer_size, dtype=torch.float,  device=self.device, requires_grad=False)
        def add_value(self, new_value):
            self.buffer = torch.cat((self.buffer[self.vector_size:], new_value), dim=-1)

        def get_buffer(self):
            return self.buffer

def skew_symm(_v):
    x, y, z = _v[0], _v[1], _v[2]
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

def quat2rot(_q):
    # print("quat: ", _q)
    q0 = _q[3]
    qv = np.reshape(_q[0:3], (3, 1))
    return (2 * q0**2 - 1) * np.eye(3) + 2 * q0 * skew_symm(qv.reshape((3, ))) + 2 * qv @ qv.T

class Executor:
    def __init__(self, name):
        self.m_name = name
        self.device = "cpu"
        path_label = "/home/meshin/dev/walk-these-ways/runs/gait-conditioned-agility/2024-02-18/train/211214.073611"
        self.policy = Policy(path_label)
        self.loop_rate = 50.
        self.dt = 1./self.loop_rate
        self.comm_node = QuadROSComm(name)
        self.comm_node.setLoopRate(200.)
        try:
            self.comm_node.start_thread()
            print("Communication thread started...")
        except:
            print("Error starting communication thread...")
        
        self.joint_cmd = JointData()
        self.sensor_data = SensorData()
        
        self.num_commands = 15
        self.num_observations = 70
        self.num_actions = 12
        self.obs_hist_size = 30

        self.gaits = {"pronking": [0, 0, 0],
                      "trotting": [0.5, 0, 0],
                      "bounding": [0, 0.5, 0],
                      "pacing": [0, 0, 0.5]}
        self.x_vel_cmd, self.y_vel_cmd, self.yaw_vel_cmd = 1.0, 0.0, 0.0
        self.body_height_cmd = 0.34
        self.step_frequency_cmd = 3.0
        self.gait = self.gaits["trotting"]
        self.footswing_height_cmd = 0.08
        self.pitch_cmd = 0.0
        self.roll_cmd = 0.0
        self.stance_width_cmd = 0.25
        self.stance_length_cmd = 0.40
        self.commands_np = np.array([0.] * self.num_commands)
        self.commands_np[0] = self.x_vel_cmd
        self.commands_np[1] = self.y_vel_cmd
        self.commands_np[2] = self.yaw_vel_cmd
        self.commands_np[3] = self.body_height_cmd
        self.commands_np[4] = self.step_frequency_cmd
        self.commands_np[5:8] = self.gait
        self.commands_np[8] = 0.5
        self.commands_np[9] = self.footswing_height_cmd
        self.commands_np[10] = self.pitch_cmd
        self.commands_np[11] = self.roll_cmd
        self.commands_np[12] = self.stance_width_cmd
        self.commands_np[13] = self.stance_length_cmd
        self.default_joint_pos = np.array([0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1., -1.5, -0.1, 1., -1.5], dtype=np.float64)
        self.joint_pos_scale = 1.0
        self.joint_vel_scale = 0.05
        self.actions_scale = 0.25
        self.hip_scale_reduction = 0.5
        self.clip_observations = 100.
        self.clip_actions = 100.

        # class obs_scales:
        self.obs_scales_lin_vel = 2.0
        self.obs_scales_ang_vel = 0.25
        self.obs_scales_dof_pos = 1.0
        self.obs_scales_dof_vel = 0.05
        # self.obs_scales_imu = 0.1
        # self.obs_scales_height_measurements = 5.0
        # self.obs_scales_friction_measurements = 1.0
        self.obs_scales_body_height_cmd = 2.0
        self.obs_scales_gait_phase_cmd = 1.0
        self.obs_scales_gait_freq_cmd = 1.0
        self.obs_scales_footswing_height_cmd = 0.15
        self.obs_scales_body_pitch_cmd = 0.3
        self.obs_scales_body_roll_cmd = 0.3
        self.obs_scales_stance_width_cmd = 1.0
        self.obs_scales_stance_length_cmd = 1.0
        self.obs_scales_aux_reward_cmd = 1.0
        self.commands_scale_np = np.array([self.obs_scales_lin_vel, self.obs_scales_lin_vel, self.obs_scales_ang_vel,
                                            self.obs_scales_body_height_cmd, self.obs_scales_gait_freq_cmd,
                                            self.obs_scales_gait_phase_cmd, self.obs_scales_gait_phase_cmd,
                                            self.obs_scales_gait_phase_cmd, self.obs_scales_gait_phase_cmd,
                                            self.obs_scales_footswing_height_cmd, self.obs_scales_body_pitch_cmd,
                                            self.obs_scales_body_roll_cmd, self.obs_scales_stance_width_cmd,
                                           self.obs_scales_stance_length_cmd, self.obs_scales_aux_reward_cmd])

        # TODO: Using decimation might also be required. Will look into this

        self.init_torch_buffers()
    
    # def __del__(self):
    #     self.comm_node.destroy_node()
    
    def init_torch_buffers(self):
        self.observations = torch.zeros(self.num_observations, dtype=torch.float, device=self.device, requires_grad=False)
        self.observation_history_buffer = TorchBuffer(vector_size=self.num_observations, history_size=self.obs_hist_size)
        self.projected_gravity = torch.zeros(3, dtype=torch.float, device=self.device, requires_grad=False)
        self.commands = torch.zeros(self.num_commands, dtype=torch.float, device=self.device, requires_grad=False)
        self.commands = torch.from_numpy(self.commands_np).float()
        self.commands_scale = torch.from_numpy(self.commands_scale_np).float()
        self.dof_pos = torch.zeros(12, dtype=torch.float, device=self.device, requires_grad=False)
        self.dof_vel = torch.zeros(12, dtype=torch.float, device=self.device, requires_grad=False)
        self.default_dof_pos = torch.from_numpy(self.default_joint_pos).float()
        self.clock_inputs = torch.zeros(4, dtype=torch.float, device=self.device, requires_grad=False)
        self.actions = torch.zeros(self.num_actions, dtype=torch.float, device=self.device, requires_grad=False)
        self.last_actions = torch.zeros_like(self.actions)
        self.gait_indices = torch.zeros(1, dtype=torch.float, device=self.device, requires_grad=False)

    def get_action(self, obs_hist):
        self.last_actions = self.actions
        self.actions = self.policy.forward(obs_hist)
        self.actions = torch.clip(self.actions, -self.clip_actions, self.clip_actions)
        return self.actions

    def update_sensor_data(self):
        self.sensor_data = self.comm_node.getSensorData()
    
    def update_joint_cmd_from_action(self):
        actions_scaled = (self.actions * self.actions_scale).detach().numpy()
        actions_scaled[[0, 3, 6, 9]] *= self.hip_scale_reduction
        joint_pos_target = actions_scaled + self.default_joint_pos
        joint_vel_target = np.array([0.] * 12)

        self.joint_cmd.q = joint_pos_target
        self.joint_cmd.qd = joint_vel_target
        self.joint_cmd.tau = [0.] * 12
        self.joint_cmd.kp = [20.] * 12
        self.joint_cmd.kd = [0.5] * 12
        self.comm_node.set_joint_command(self.joint_cmd)

    def update_gait_clock(self):
        frequencies = self.commands[4]
        phases = self.commands[5]
        offsets = self.commands[6]
        bounds = self.commands[7]
        durations = self.commands[8]
        self.gait_indices = torch.remainder(self.gait_indices + self.dt * frequencies, 1.0)

        foot_indices = [self.gait_indices + phases + offsets + bounds,
                        self.gait_indices + offsets,
                        self.gait_indices + bounds,
                        self.gait_indices + phases]

        self.foot_indices = torch.remainder(torch.cat([foot_indices[i].unsqueeze(1) for i in range(4)], dim=1), 1.0)

        self.clock_inputs[0] = torch.sin(2 * np.pi * foot_indices[0])
        self.clock_inputs[1] = torch.sin(2 * np.pi * foot_indices[1])
        self.clock_inputs[2] = torch.sin(2 * np.pi * foot_indices[2])
        self.clock_inputs[3] = torch.sin(2 * np.pi * foot_indices[3])

    def get_observations(self):
        Rot = quat2rot(self.sensor_data.quat)
        # print("Rot: ", Rot)
        # gravity = torch.Tensor([0, 0, -9.8]).to(self.device)
        # g_norm = gravity.unsqueeze(0) / torch.norm(gravity)
        gravity = np.array([0., 0., -9.81])
        g_norm = gravity / np.linalg.norm(gravity)
        proj_g = Rot.T @ g_norm
        # print("proj_g: ", proj_g)
        # js = self.comm_node.getJointAngles()
        # joint_pos, joint_vel = js['q'], js['qd']

        self.projected_gravity = torch.from_numpy(proj_g).float()
        self.dof_pos = torch.from_numpy(self.sensor_data.q).float()
        self.dof_vel = torch.from_numpy(self.sensor_data.qd).float()

        # TODO: implement function to calculate clock_inputs
        self.update_gait_clock()

        self.observations = torch.cat((self.projected_gravity,
                                      self.commands * self.commands_scale,
                                    (self.dof_pos - self.default_dof_pos) * self.obs_scales_dof_pos,
                                    self.dof_vel * self.obs_scales_dof_vel,
                                    self.actions, 
                                    self.last_actions, 
                                    self.clock_inputs), 
                                        dim=-1)
        
        self.observations = torch.clip(self.observations, -self.clip_observations, self.clip_observations)
        # print("Observation: ", self.observations)
        # print("observation size: ", self.observation.size())
        self.observation_history_buffer.add_value(self.observations)
    
    def setLoopRate(self, rate):
        self.loop_rate = rate
        self.dt = 1./rate
    
    def step(self):
        # Get sensor values
        # self.sensor_data = self.comm_node.getSensorData()
        self.update_sensor_data()
        # Get/Update observations based on sensor values
        self.get_observations()
        # Get/Update actions based on sensor values
        # print("Buffer size: ", self.observation_history_buffer.get_buffer().size())
        # print("Observation buffer data type: ", self.observation_history_buffer.get_buffer().dtype)
        self.get_action(self.observation_history_buffer.get_buffer())

        # Update joint command based on actions
        if (self.iters > 0):
            self.update_joint_cmd_from_action()
        self.iters = self.iters + 1
    
    def run(self):
        self.iters = 0
        while(True):
            try:
                self.step()
                time.sleep(self.dt)
            except KeyboardInterrupt:
                break


def main(args=None):
    rclpy.init(args=args)

    executor = Executor("go2")

    executor.setLoopRate(50.)
    executor.run()

    executor.comm_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
