from collections import deque
from typing import Optional

import numpy as np
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.sensor import ContactSensor, IMUSensor

import Go2Py
from Go2Py.sim.isaacsim.lcm_types.unitree_lowlevel import UnitreeLowState


class UnitreeGo2(Articulation):
    def __init__(
        self,
        prim_path: str,
        name: str = "go2_quadruped",
        physics_dt: Optional[float] = 1 / 400.0,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        usd_path: Optional[str] = None,
    ) -> None:
        """
        [Summary]

        initialize robot, set up sensors and interfaces

        Args:
            prim_path {str} -- prim path of the robot on the stage
            name {str} -- name of the quadruped
            physics_dt {float} -- physics downtime of the controller
            position {np.ndarray} -- position of the robot
            orientation {np.ndarray} -- orientation of the robot
        """
        self._prim_path = prim_path
        prim = define_prim(self._prim_path, "Xform")
        prim.GetReferences().AddReference(Go2Py.GO2_USD_PATH)
        self.usd_path = usd_path
        if self.usd_path is None:
            prim.GetReferences().AddReference(Go2Py.GO2_USD_PATH)
        else:
            prim.GetReferences().AddReference(self.usd_path)

        super().__init__(
            prim_path=self._prim_path,
            name=name,
            position=position,
            orientation=orientation,
        )

        # contact sensor setup
        self.feet_order = ["FL", "RL", "FR", "RR"]
        self.feet_path = [
            self._prim_path + "/FL_foot",
            self._prim_path + "/FR_foot",
            self._prim_path + "/RL_foot",
            self._prim_path + "/RR_foot",
        ]

        self.color = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1), (1, 1, 0, 1)]
        self._contact_sensors = [None] * 4
        for i in range(4):
            self._contact_sensors[i] = ContactSensor(
                prim_path=self.feet_path[i] + "/sensor",
                min_threshold=0,
                max_threshold=1000000,
                radius=0.043,
                dt=physics_dt,
            )

        self.foot_force = np.zeros(4)
        self.enable_foot_filter = True
        self._FILTER_WINDOW_SIZE = 20
        self._foot_filters = [deque(), deque(), deque(), deque()]
        # imu sensor setup
        self.imu_path = self._prim_path + "/imu_link"
        self._imu_sensor = IMUSensor(
            prim_path=self.imu_path + "/imu_sensor",
            name="imu",
            dt=physics_dt,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
        )
        self.accel = np.zeros((3,))
        self.gyro = np.zeros((3,))

        # Translation maps between Isaac and Bullet
        # self.bullet_joint_order = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        #                            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        #                            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        #                            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']

        self.bullet_joint_order = [
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
        self.isaac_joint_order = [
            "FL_hip_joint",
            "FR_hip_joint",
            "RL_hip_joint",
            "RR_hip_joint",
            "FL_thigh_joint",
            "FR_thigh_joint",
            "RL_thigh_joint",
            "RR_thigh_joint",
            "FL_calf_joint",
            "FR_calf_joint",
            "RL_calf_joint",
            "RR_calf_joint",
        ]

        self.isaac_name_2_index = {s: i for i, s in enumerate(self.isaac_joint_order)}
        self.bullet_name_2_index = {s: i for i, s in enumerate(self.bullet_joint_order)}

        self.to_bullet_index = np.array(
            [self.isaac_name_2_index[id] for id in self.bullet_joint_order]
        )
        self.to_isaac_index = np.array(
            [self.bullet_name_2_index[id] for id in self.isaac_joint_order]
        )
        self.tau_est = np.zeros((12,))
        self.state = UnitreeLowState()
        self.init_pos = np.array([0.0, 0.0, 0.6])
        self.init_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.init_joint_pos = np.array(
            [0.2, 0.8, -1.5, -0.2, 0.8, -1.5, 0.2, 1.0, -1.6, -0.2, 1.0, -1.6]
        )
        return

    def toIsaacOrder(self, x):
        return x[self.to_isaac_index, ...]

    def toBulletOrder(self, x):
        return x[self.to_bullet_index, ...]

    def setState(self, pos, quat, q) -> None:
        """[Summary]

        Set the kinematic state of the robot.

        Args:
            pos  {ndarray} -- The position of the robot (x, y, z)
            quat {ndarray} -- The orientation of the robot (qx, qy, qz, qw)
            q    {ndarray} -- Joint angles of the robot in standard Pinocchio order

        Raises:
            RuntimeError: When the DC Toolbox interface has not been configured.
        """
        self.set_world_pose(position=pos, orientation=quat[[3, 0, 1, 2]])
        self.set_linear_velocity(np.zeros((3,)))
        self.set_angular_velocity(np.zeros((3,)))

        self.set_joint_positions(
            positions=np.asarray(self.toIsaacOrder(q), dtype=np.float32)
        )
        self.set_joint_velocities(
            velocities=np.asarray(self.toIsaacOrder(np.zeros((12,))), dtype=np.float32)
        )
        self.set_joint_efforts(np.zeros_like(q))
        return

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]

        initialize dc interface, set up drive mode and initial robot state

        """
        super().initialize(physics_sim_view=physics_sim_view)
        self.get_articulation_controller().set_effort_modes("force")
        self.get_articulation_controller().switch_control_mode("effort")
        self.setState(self.init_pos, self.init_quat, self.init_joint_pos)
        for i in range(4):
            self._contact_sensors[i].initialize()
        return

    def readContactSensor(self) -> None:
        """[summary]

        Updates processed contact sensor data from the robot feet, store them in member variable foot_force
        """
        # Order: FL, RL, FR, RR
        for i in range(len(self.feet_path)):
            frame = self._contact_sensors[i].get_current_frame()
            if "force" in frame:
                if self.enable_foot_filter:
                    self._foot_filters[i].append(frame["force"])
                    if len(self._foot_filters[i]) > self._FILTER_WINDOW_SIZE:
                        self._foot_filters[i].popleft()
                    self.foot_force[i] = np.mean(self._foot_filters[i])

                else:
                    self.foot_force[i] = frame["force"]
        return self.foot_force

    def readIMUSensor(self) -> None:
        """[summary]

        Updates processed imu sensor data from the robot body, store them in member variable base_lin and ang_vel
        """
        frame = self._imu_sensor.get_current_frame()
        self.accel = frame["lin_acc"]
        self.gyro = frame["ang_vel"]
        return self.accel, self.gyro

    def readStates(self):
        contact_forces = self.readContactSensor()
        accel, gyro = self.readIMUSensor()

        # joint pos and vel from the DC interface
        joint_state = super().get_joints_state()
        joint_pos = self.toBulletOrder(joint_state.positions)
        joint_vel = self.toBulletOrder(joint_state.velocities)

        # base frame
        base_pose = self.get_world_pose()
        base_pos = base_pose[0]
        base_quat = base_pose[1][[1, 2, 3, 0]]
        base_lin_vel = self.get_linear_velocity()
        base_ang_vel = self.get_angular_velocity()

        # assign to state objects
        self.state.accel = accel.tolist()
        self.state.gyro = gyro.tolist()
        self.state.q = joint_pos.tolist()
        self.state.dq = joint_vel.tolist()
        self.state.tau_est = self.tau_est
        self.state.quaternion = base_quat.tolist()
        self.state.gt_pos = base_pos.tolist()
        self.state.gt_quat = base_quat.tolist()
        self.state.gt_lin_vel = base_lin_vel.tolist()
        self.state.gt_ang_vel = base_ang_vel.tolist()
        self.state.contact_state = contact_forces.tolist()
        # projected_gravity = pp.SO3(base_quat).matrix().T @ np.array(
        #     [0.0, 0.0, -1.0]
        # ).reshape(3, 1)
        # self.state.gravity = (
        #     projected_gravity.squeeze().numpy().astype(np.float32).tolist()
        # )
        return self.state

    def setCommands(self, cmd):
        """[summary]
        sets the joint torques
        Argument:
            action {np.ndarray} -- Joint torque command
        """
        q = np.array(self.state.q)
        dq = np.array(self.state.dq)
        kp = np.array(cmd.kp)
        kd = np.array(cmd.kd)
        tau_ff = np.array(cmd.tau_ff)
        q_des = np.array(cmd.q_des)
        dq_des = np.array(cmd.dq_des)
        tau = (q_des - q) * kp + (dq_des - dq) * kd + tau_ff
        self.set_joint_efforts(np.asarray(self.toIsaacOrder(tau), dtype=np.float32))
        self.tau_est = tau
        return

    def step(self, cmd):
        self.readStates()
        self.setCommands(cmd)
        return self.state
