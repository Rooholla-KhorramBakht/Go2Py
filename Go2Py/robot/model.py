import os
from Go2Py import ASSETS_PATH
import pinocchio as pin
import numpy as np
urdf_path = os.path.join(ASSETS_PATH, 'urdf/go2.urdf')
urdf_root_path = os.path.join(ASSETS_PATH, 'urdf')

class FrictionModel:
    def __init__(self, mu_v=0.1, Fs=0.3, Vs=0.5, temperature=0.1):
        self.mu_v = mu_v
        self.Fs = Fs
        self.Vs = Vs 
        self.temperature = temperature

    def __call__(self, dq):
        # tau_sticktion = self.Fs*np.exp(-(np.abs(dq)/self.Vs)**2)*self.softSign(dq, temperature=self.temperature)
        tau_sticktion = self.Fs*self.softSign(dq, temperature=self.temperature)
        tau_viscose = self.mu_v*dq
        return tau_sticktion+tau_viscose

    def softSign(self, u, temperature=0.1):
        return np.tanh(u/temperature)

class Go2Model:
    """
    A model class for the Go2 quadruped robot using the Pinocchio library.

    Attributes:
        robot (pin.RobotWrapper): The Pinocchio RobotWrapper instance for the Go2 robot.
        data (pin.Model.Data): The data structure used by Pinocchio for computations.
        ef_frames (list): List of end-effector frame names.
        dq_reordering_idx (np.ndarray): Index array for reordering the joint velocity vector.
        q_reordering_idx (np.ndarray): Index array for reordering the joint position vector.
        ef_J_ (dict): A dictionary storing the Jacobians for the end-effector frames.
    """

    def __init__(self):
        """
        Initializes the Go2Model class by loading the URDF, setting up the robot model, and calculating initial dimensions.
        """
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path, urdf_root_path, pin.JointModelFreeFlyer())
        self.data = self.robot.data
        # Standing joint configuration in Unitree Joint order
        self.ef_frames = ['FR_foot', 'FL_foot', 'RR_foot', 'RL_foot']
        self.dq_reordering_idx = np.array([0, 1, 2, 3, 4, 5,
                                           9, 10, 11, 6, 7, 8, 15, 16, 17, 12, 13, 14])
        self.q_reordering_idx = np.array([9, 10, 11, 6, 7, 8, 15, 16, 17, 12, 13, 14]) - 6
        self.ef_Jb_ = {}
        self.ef_Jw_ = {}

        ID_FL_HAA = self.robot.model.getFrameId('FL_hip_joint')
        ID_FR_HAA = self.robot.model.getFrameId('FR_hip_joint')
        ID_RL_HAA = self.robot.model.getFrameId('RL_hip_joint')
        ID_RR_HAA = self.robot.model.getFrameId('RR_hip_joint')
        ID_FL_HFE = self.robot.model.getFrameId('FL_thigh_joint')
        ID_FR_HFE = self.robot.model.getFrameId('FR_thigh_joint')
        ID_RL_HFE = self.robot.model.getFrameId('RL_thigh_joint')
        ID_RR_HFE = self.robot.model.getFrameId('RR_thigh_joint')
        ID_FL_KFE = self.robot.model.getFrameId('FL_calf_joint')
        ID_FR_KFE = self.robot.model.getFrameId('FR_calf_joint')
        ID_RL_KFE = self.robot.model.getFrameId('RL_calf_joint')
        ID_RR_KFE = self.robot.model.getFrameId('RR_calf_joint')
        ID_FR_FOOT = self.robot.model.getFrameId('FR_foot')

        q_neutral = np.asarray([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.robot.framesForwardKinematics(q_neutral)

        self.h = np.linalg.norm(
            self.robot.data.oMf[ID_FR_HAA].translation -
            self.robot.data.oMf[ID_RR_HAA].translation)
        self.b = np.linalg.norm(
            self.robot.data.oMf[ID_FR_HAA].translation -
            self.robot.data.oMf[ID_FL_HAA].translation)
        self.l1 = np.linalg.norm(
            self.robot.data.oMf[ID_FR_HAA].translation -
            self.robot.data.oMf[ID_FR_HFE].translation)
        self.l2 = np.linalg.norm(
            self.robot.data.oMf[ID_FR_HFE].translation -
            self.robot.data.oMf[ID_FR_KFE].translation)
        self.l3 = np.linalg.norm(
            self.robot.data.oMf[ID_FR_KFE].translation -
            self.robot.data.oMf[ID_FR_FOOT].translation)
        self.M_ = None
        self.Minv_ = None
        self.nle_ = None
        self.g_ = None
        # print(self.robot.data.oMf[ID_FR_HAA].translation - self.robot.data.oMf[ID_RR_HAA].translation)
        # print(self.h)
        # print(self.b)
        # print(self.l1)
        # print(self.l2)
        # print(self.l3)

    def inverseKinematics(self, T, feet_pos):
        """
        Calculates the inverse kinematics for the robot given a desired state.

        Args:
            T (np.ndarray): The 4x4 homogeneous transformation representing the pose of the base_link in the world frame
            x (np.ndarray): A numpy array of size 12 representing foot positions in world frame in FR, FL, RR, RL order.

        Returns:
            np.ndarray: A numpy array of size 12 representing the joint angles of the legs.
        """
        rB = np.asarray(T[0:3, -1])  # Base position (3D vector)
        R = T[:3, :3]                # Body orientation (quaternion converted to rotation matrix)

        sx = [1, 1, -1, -1]
        sy = [-1, 1, -1, 1]

        joint_angles = np.zeros(12)

        for i in range(4):
            r_HB = np.array([sx[i] * self.h / 2, sy[i] * self.b / 2, 0])  # Hip offset (3D vector)
            rf = np.asarray(feet_pos[3 * i:3 * i + 3])  # Foot position (3D vector)
            r_fH = R.T @ (rf - rB) - r_HB  # Foot relative to hip in body frame (3D vector)

            x = r_fH[0]
            y = r_fH[1]
            z = r_fH[2]
            et = y**2 + z**2 - self.l1**2

            # Theta 3 calculation
            c3 = (x**2 + et - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
            s3 = -np.sqrt(1 - c3**2)
            t3 = np.arctan2(s3, c3)

            # Theta 2 calculation
            k1 = self.l2 + self.l3 * c3
            k2 = self.l3 * s3
            r1 = np.sqrt(k1**2 + k2**2)
            t2 = np.arctan2(-x / r1, np.sqrt(et) / r1) - np.arctan2(k2 / r1, k1 / r1)

            # Theta 1 calculation
            zv = self.l2 * np.cos(t2) + self.l3 * np.cos(t2 + t3)
            m1 = sy[i] * self.l1
            m2 = -zv
            r2 = np.sqrt(m1**2 + m2**2)
            t1 = np.arctan2(z / r2, y / r2) - np.arctan2(m2 / r2, m1 / r2)

            joint_angles[3 * i:3 * i + 3] = np.array([t1, t2, t3])

        # TODO: Implement joint axis direction multiplication from URDF

        return joint_angles

    def forwardKinematics(self, T, q):
        """
        Computes the forward kinematics for the robot given a transformation matrix and joint configuration.

        Args:
            T (np.ndarray): 4x4 Transformation matrix representing the base pose of the robot.
            q (np.ndarray): A numpy array of size 12 representing the joint configurations in FR, FL, RR, RL order.

        Returns:
            dict: A dictionary containing the poses of specified frames in the robot.
        """
        q_ = np.hstack([pin.SE3ToXYZQUATtuple(pin.SE3(T)), q[self.q_reordering_idx]])
        self.robot.framesForwardKinematics(q_)
        ef_frames = ['base_link', 'FR_foot', 'FL_foot', 'RR_foot', 'RL_foot']
        data = {}
        return {frame: self.robot.data.oMf[self.robot.model.getFrameId(frame)].homogeneous
                for frame in ef_frames}

    def updateKinematics(self, q):
        """
        Updates the kinematic states.

        Args:
            q (np.ndarray): A numpy array of size 19 representing the [x, y, z, qx, qy, qz, qw] and joint configurations in FR, FL, RR, RL order.
        """
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)\

        for ef_frame in self.ef_frames:
            Jw = self.robot.getFrameJacobian(
                self.robot.model.getFrameId(ef_frame),
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            Jb = self.robot.getFrameJacobian(
                self.robot.model.getFrameId(ef_frame),
                pin.ReferenceFrame.LOCAL)
            self.ef_Jw_[ef_frame] = Jw[:, self.dq_reordering_idx]
            self.ef_Jb_[ef_frame] = Jb[:, self.dq_reordering_idx]

    def updateKinematicsPose(self, q, T):
        """
        Updates the kinematic states.

        Args:
            q (np.ndarray): A numpy array of size 12 representing the joint configurations in FR, FL, RR, RL order.
            T (np.ndarray): 4x4 Transformation matrix representing the base pose of the robot.
        """
        q_ = np.hstack([pin.SE3ToXYZQUATtuple(pin.SE3(T)), q[self.q_reordering_idx]])
        self.robot.computeJointJacobians(q_)
        self.robot.framesForwardKinematics(q_)\

        for ef_frame in self.ef_frames:
            Jw = self.robot.getFrameJacobian(
                self.robot.model.getFrameId(ef_frame),
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            Jb = self.robot.getFrameJacobian(
                self.robot.model.getFrameId(ef_frame),
                pin.ReferenceFrame.LOCAL)
            self.ef_Jw_[ef_frame] = Jw[:, self.dq_reordering_idx]
            self.ef_Jb_[ef_frame] = Jb[:, self.dq_reordering_idx]

    def updateDynamics(self, q, dq):
        """
        Updates the dynamical states.

        Args:
            q (np.ndarray): A numpy array of size 19 representing the [x, y, z, qx, qy, qz, qw] and joint configurations in FR, FL, RR, RL order.
            dq (np.ndarray): A numpy array of size 18 representing the [vx, vy, vz, wx, wy, wz] and joint configurations in FR, FL, RR, RL order.
        """
        self.robot.centroidalMomentum(q, dq)
        self.nle_ = self.robot.nle(q, dq)[self.dq_reordering_idx]
        self.g_ = self.robot.gravity(q)[self.dq_reordering_idx]
        self.M_ = self.robot.mass(q)[self.dq_reordering_idx, :]
        self.M_ = self.M_[:, self.dq_reordering_idx]
        self.Minv_ = pin.computeMinverse(
            self.robot.model, self.robot.data, q)[
            self.dq_reordering_idx, :]
        self.Minv_ = self.Minv_[:, self.dq_reordering_idx]

    def updateAll(q, dq):
        """
        Updates the dynamic and kinematic parameters based on the given joint configurations and velocities.

        Args:
            q (np.ndarray): A numpy array of size 19 representing the [x, y, z, qx, qy, qz, qw] and joint configurations in FR, FL, RR, RL order.
            dq (np.ndarray): A numpy array of size 18 representing the [vx, vy, vz, wx, wy, wz] and joint configurations in FR, FL, RR, RL order.
        """
        self.updateKinematics(q)
        self.updateDynamics(q, dq)

    def updateAllPose(self, q, dq, T, v):
        """
        Updates the dynamic and kinematic parameters based on the given joint configurations and velocities.

        Args:
            q (np.ndarray): A numpy array of size 12 representing the joint configurations in FR, FL, RR, RL order.
            dq (np.ndarray): A numpy array of size 12 representing the joint velocities in FR, FL, RR, RL order.
            T (np.ndarray): 4x4 Transformation matrix representing the base pose of the robot.
            v (np.ndarray): A numpy array of size 6 representing the base velocity in body frame [v, w].
        """
        q_ = np.hstack([pin.SE3ToXYZQUATtuple(pin.SE3(T)), q[self.q_reordering_idx]])
        dq_ = np.hstack([v, dq[self.q_reordering_idx]])
        self.updateKinematics(q_)
        self.updateDynamics(q_, dq_)

    def getInfo(self):
        """
        Retrieves the current dynamics and kinematic information of the robot.

        Returns:
            dict: A dictionary containing the robot's mass matrix, inverse mass matrix, non-linear effects, gravity vector, and Jacobians for the end-effectors.
        """
        return {
            'M': self.M_,
            'Minv': self.Minv_,
            'nle': self.nle_,
            'g': self.g_,
            'J_w': self.ef_Jw_,
            'J_b': self.ef_Jb_,
        }

    def getGroundReactionForce(self, tau_est, body_acceleration=None):
        if body_acceleration is None:
            grf = {key: np.linalg.pinv(
                self.ef_Jw_[key][:3, 6:].T) @ (tau_est.squeeze() - self.nle_[6:]) for key in self.ef_Jw_.keys()}
        else:
            raise NotImplementedError(
                "Ground reaction force with body dynamics is not implemented")
        return grf
