import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import onnxruntime

class RunningMeanStd(nn.Module):
    def __init__(self, shape = (), epsilon=1e-08):
        super(RunningMeanStd, self).__init__()
        self.register_buffer("running_mean", torch.zeros(shape))
        self.register_buffer("running_var", torch.ones(shape))
        self.register_buffer("count", torch.ones(()))

        self.epsilon = epsilon

    def forward(self, obs, update = True):
        if update:
            self.update(obs)

        return (obs - self.running_mean) / torch.sqrt(self.running_var + self.epsilon)

    def update(self, x):
        """Updates the mean, var and count from a batch of samples."""
        batch_mean = torch.mean(x, dim=0)
        batch_var = torch.var(x, correction=0, dim=0)
        batch_count = x.shape[0]
        self.update_from_moments(batch_mean, batch_var, batch_count)

    def update_from_moments(self, batch_mean, batch_var, batch_count):
        """Updates from batch mean, variance and count moments."""
        self.running_mean, self.running_var, self.count = update_mean_var_count_from_moments(
            self.running_mean, self.running_var, self.count, batch_mean, batch_var, batch_count
        )

def update_mean_var_count_from_moments(
    mean, var, count, batch_mean, batch_var, batch_count
):
    """Updates the mean, var and count using the previous mean, var, count and batch values."""
    delta = batch_mean - mean
    tot_count = count + batch_count

    new_mean = mean + delta * batch_count / tot_count
    m_a = var * count
    m_b = batch_var * batch_count
    M2 = m_a + m_b + torch.square(delta) * count * batch_count / tot_count
    new_var = M2 / tot_count
    new_count = tot_count

    return new_mean, new_var, new_count

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer

class Agent(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor_memory = nn.GRU(45, 256, batch_first=True)
        self.critic_memory = nn.GRU(45, 256, batch_first=True)
        self.critic = nn.Sequential(
            layer_init(nn.Linear(45 + 256, 512)),
            nn.ELU(),
            layer_init(nn.Linear(512, 256)),
            nn.ELU(),
            layer_init(nn.Linear(256, 128)),
            nn.ELU(),
            layer_init(nn.Linear(128, 1), std=1.0),
        )
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(45 + 256, 512)),
            nn.ELU(),
            layer_init(nn.Linear(512, 256)),
            nn.ELU(),
            layer_init(nn.Linear(256, 128)),
            nn.ELU(),
            layer_init(nn.Linear(128, 12), std=0.01),
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, 12))

        self.obs_rms = RunningMeanStd(shape = (45,))
        self.value_rms = RunningMeanStd(shape = ())

    def forward(self, x, ac_hidden_in):
        x = self.obs_rms(x, update = False)
        ac_out, ac_hidden_out = self.actor_memory(x, ac_hidden_in)
        h = torch.cat([x, ac_out], dim = -1)
        action_mean = self.actor_mean(h)
        return action_mean, ac_hidden_out

class Policy:
    def __init__(self, checkpoint_path):
        self.agent = Agent()
        actor_sd = torch.load(checkpoint_path, map_location="cpu")
        self.agent.load_state_dict(actor_sd)
        onnx_file_name = checkpoint_path.replace(".pt", ".onnx")

        dummy_input = torch.randn(1, 1, 45)
        dummy_hidden = torch.randn(1, 1, 256)
        with torch.no_grad():
            torch_out, torch_hidden_out = self.agent(dummy_input, dummy_hidden)

        torch.onnx.export(
            self.agent,                          # The model being converted
            (dummy_input, dummy_hidden),    # An example input for the model
            onnx_file_name,                 # Output file name
            export_params=True,             # Store trained parameter weights inside the model file
            opset_version=11,               # ONNX version (opset) to export to, adjust as needed
            do_constant_folding=True,       # Whether to perform constant folding optimization
            input_names=['input', 'hidden_in'],          # Name of the input in the ONNX graph (can be customized)
            output_names=['action', 'hidden_out'],  # Name of the output (assuming get_action and get_value are key outputs)
        )

        self.ort_session = onnxruntime.InferenceSession(onnx_file_name, providers=["CPUExecutionProvider"])
        ort_inputs = {'input': dummy_input.numpy(), 'hidden_in': dummy_hidden.numpy()}
        ort_outs = self.ort_session.run(None, ort_inputs)
        action, hidden_out = ort_outs[0], ort_outs[1]
        np.testing.assert_allclose(torch_out.numpy(), action, rtol=1e-03, atol=1e-05)
        np.testing.assert_allclose(torch_hidden_out.numpy(), hidden_out, rtol=1e-03, atol=1e-05)
        print("Exported model has been tested with ONNXRuntime, and the result looks good!")

    def __call__(self, obs, info):
        #with torch.no_grad():
        #    action = self.agent.get_action(self.agent.obs_rms(obs.unsqueeze(0), update = False))
        ort_inputs = {'input': obs[np.newaxis].astype(np.float32)}
        ort_outs = self.ort_session.run(None, ort_inputs)
        return ort_outs[0]

class CommandInterface:
    def __init__(self, limits=None):
        self.limits = limits
        self.x_vel_cmd, self.y_vel_cmd, self.yaw_vel_cmd = 0.0, 0.0, 0.0

    def get_command(self):
        command = np.zeros((3,))
        command[0] = self.x_vel_cmd
        command[1] = self.y_vel_cmd
        command[2] = self.yaw_vel_cmd
        return command, False


class CaTAgent:
    def __init__(self, command_profile, robot):
        self.robot = robot
        self.command_profile = command_profile
        # self.lcm_bridge = LCMBridgeClient(robot_name=self.robot_name)
        self.sim_dt = 0.001
        self.decimation = 20
        self.dt = self.sim_dt * self.decimation
        self.timestep = 0

        self.device = "cpu"

        joint_names = [
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
        ]

        policy_joint_names = joint_names

        unitree_joint_names = [
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ]

        policy_to_unitree_map = []
        unitree_to_policy_map = []
        for i, policy_joint_name in enumerate(policy_joint_names):
            id = np.where([name == policy_joint_name for name in unitree_joint_names])[0][0]
            policy_to_unitree_map.append((i, id))
        self.policy_to_unitree_map = np.array(policy_to_unitree_map).astype(np.uint32)

        for i, unitree_joint_name in enumerate(unitree_joint_names):
            id = np.where([name == unitree_joint_name for name in policy_joint_names])[0][0]
            unitree_to_policy_map.append((i, id))
        self.unitree_to_policy_map = np.array(unitree_to_policy_map).astype(np.uint32)

        default_joint_angles = {
            "FL_hip_joint": 0.1,
            "RL_hip_joint": 0.1,
            "FR_hip_joint": -0.1,
            "RR_hip_joint": -0.1,
            "FL_thigh_joint": 0.8,
            "RL_thigh_joint": 1.0,
            "FR_thigh_joint": 0.8,
            "RR_thigh_joint": 1.0,
            "FL_calf_joint": -1.5,
            "RL_calf_joint": -1.5,
            "FR_calf_joint": -1.5,
            "RR_calf_joint": -1.5
        }
        self.default_dof_pos = np.array(
            [
                default_joint_angles[name]
                for name in joint_names
            ]
        )

        self.default_dof_pos = self.default_dof_pos

        self.p_gains = np.zeros(12)
        self.d_gains = np.zeros(12)
        for i in range(12):
            self.p_gains[i] = 40. # 20.0
            self.d_gains[i] = 1. # 0.5

        print(f"p_gains: {self.p_gains}")

        self.commands = np.zeros(3)
        self.actions = np.zeros((1, 12))
        self.last_actions = np.zeros((1,12))
        self.gravity_vector = np.zeros(3)
        self.dof_pos = np.zeros(12)
        self.dof_vel = np.zeros(12)
        self.body_linear_vel = np.zeros(3)
        self.body_angular_vel = np.zeros(3)
        self.joint_pos_target = np.zeros(12)
        self.joint_vel_target = np.zeros(12)
        self.prev_joint_acc = None
        self.torques = np.zeros(12)
        self.contact_state = np.ones(4)
        self.foot_contact_forces_mag = np.zeros(4)
        self.prev_foot_contact_forces_mag = np.zeros(4)
        self.test = 0

    def wait_for_state(self):
        # return self.lcm_bridge.getStates(timeout=2)
        pass

    def get_obs(self):
        cmds, reset_timer = self.command_profile.get_command()
        self.commands[:] = cmds

        # self.state = self.wait_for_state()
        joint_state = self.robot.getJointStates()
        if joint_state is not None:
            self.gravity_vector = self.robot.getGravityInBody()
            self.prev_dof_pos = self.dof_pos.copy()
            self.dof_pos = np.array(joint_state['q'])[self.unitree_to_policy_map[:, 1]]
            self.prev_dof_vel = self.dof_vel.copy()
            self.dof_vel = np.array(joint_state['dq'])[self.unitree_to_policy_map[:, 1]]
            self.body_angular_vel = self.robot.getIMU()["gyro"]
            
            try:
                self.foot_contact_forces_mag = self.robot.getFootContact()
                self.body_linear_vel = self.robot.getLinVel()
            except:
                pass

        ob = np.concatenate(
            (
                self.body_angular_vel * 0.25,
                self.commands * np.array([2.0, 2.0, 0.25]),
                self.gravity_vector[:, 0],
                self.dof_pos * 1.0,
                #((self.dof_pos - self.prev_dof_pos) / self.dt) * 0.05,
                self.dof_vel * 0.05,
                self.last_actions[0]
            ),
            axis=0,
        )

        #return torch.tensor(ob, device=self.device).float()
        return ob

    def publish_action(self, action, hard_reset=False):
        # command_for_robot = UnitreeLowCommand()
        #self.joint_pos_target = (
        #    action[0, :12].detach().cpu().numpy() * 0.25
        #).flatten()
        self.joint_pos_target = (
            action[0, :12] * 0.25
        ).flatten()
        self.joint_pos_target += self.default_dof_pos
        self.joint_vel_target = np.zeros(12)
        # command_for_robot.q_des = self.joint_pos_target
        # command_for_robot.dq_des = self.joint_vel_target
        # command_for_robot.kp = self.p_gains
        # command_for_robot.kd = self.d_gains
        # command_for_robot.tau_ff = np.zeros(12)
        if hard_reset:
            command_for_robot.id = -1

        self.torques = (self.joint_pos_target - self.dof_pos) * self.p_gains + (
            self.joint_vel_target - self.dof_vel
        ) * self.d_gains
        # self.lcm_bridge.sendCommands(command_for_robot)
        self.robot.setCommands(self.joint_pos_target[self.policy_to_unitree_map[:, 1]],
                               self.joint_vel_target[self.policy_to_unitree_map[:, 1]],
                               self.p_gains[self.policy_to_unitree_map[:, 1]],
                               self.d_gains[self.policy_to_unitree_map[:, 1]],
                               np.zeros(12))

    def reset(self):
        self.actions = torch.zeros((1, 12))
        self.time = time.time()
        self.timestep = 0
        return self.get_obs()

    def step(self, actions, hard_reset=False):
        self.last_actions = self.actions[:]
        self.actions = actions
        self.publish_action(self.actions, hard_reset=hard_reset)
        # time.sleep(max(self.dt - (time.time() - self.time), 0))
        # if self.timestep % 100 == 0:
        #     print(f"frq: {1 / (time.time() - self.time)} Hz")
        self.time = time.time()
        #obs = self.get_obs()

        joint_acc = np.abs(self.prev_dof_vel - self.dof_vel) / self.dt
        if self.prev_joint_acc is None:
            self.prev_joint_acc = np.zeros_like(joint_acc)
        joint_jerk = np.abs(self.prev_joint_acc - joint_acc) / self.dt
        self.prev_joint_acc = joint_acc.copy()

        foot_contact_rate = np.abs(self.foot_contact_forces_mag - self.prev_foot_contact_forces_mag)
        self.prev_foot_contact_forces_mag = self.foot_contact_forces_mag.copy()

        infos = {
            "joint_pos": self.dof_pos[np.newaxis, :],
            "joint_vel": self.dof_vel[np.newaxis, :],
            "joint_pos_target": self.joint_pos_target[np.newaxis, :],
            "joint_vel_target": self.joint_vel_target[np.newaxis, :],
            "body_linear_vel": self.body_linear_vel[np.newaxis, :],
            "body_angular_vel": self.body_angular_vel[np.newaxis, :],
            "contact_state": self.contact_state[np.newaxis, :],
            "body_linear_vel_cmd": self.commands[np.newaxis, 0:2],
            "body_angular_vel_cmd": self.commands[np.newaxis, 2:],
            "torques": self.torques,
            "foot_contact_forces_mag": self.foot_contact_forces_mag.copy(),
            "joint_acc": joint_acc[np.newaxis, :],
            "joint_jerk": joint_jerk[np.newaxis, :],
            "foot_contact_rate": foot_contact_rate[np.newaxis, :],
        }

        self.timestep += 1
        return None, None, None, infos
