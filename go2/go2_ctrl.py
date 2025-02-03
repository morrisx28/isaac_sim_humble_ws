# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import io
from typing import List, Optional

import carb
import numpy as np
import omni
import omni.kit.commands
import torch
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.rotations import euler_to_rot_matrix, quat_to_euler_angles, quat_to_rot_matrix
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.nucleus import get_assets_root_path
from pxr import Gf


class Go2FlatTerrainPolicy:
    """The Spot quadruped"""

    def __init__(
        self,
        prim_path: str,
        name: str = "go2",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize robot and load RL policy.

        Args:
            prim_path {str} -- prim path of the robot on the stage
            name {str} -- name of the quadruped
            usd_path {str} -- robot usd filepath in the directory
            position {np.ndarray} -- position of the robot
            orientation {np.ndarray} -- orientation of the robot

        """
        self._stage = get_current_stage()
        self._prim_path = prim_path
        prim = get_prim_at_path(self._prim_path)

        assets_root_path = get_assets_root_path()
        if not prim.IsValid():
            prim = define_prim(self._prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")

                asset_path = assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"

                prim.GetReferences().AddReference(asset_path)

        self.robot = Articulation(prim_path=self._prim_path, name=name, position=position, orientation=orientation)

        self._dof_control_modes: List[int] = list()

        # Policy
        file = "/home/csl/backup/IsaacLab/logs/rsl_rl/unitree_go2_flat/2025-01-07_11-53-07/exported/policy.pt"
        self._policy = torch.jit.load(file)
        self._base_vel_lin_scale = 1
        self._base_vel_ang_scale = 1
        self._action_scale = 0.25
        self._default_joint_pos = np.array([0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5])
        self._previous_action = np.zeros(12)
        self._policy_counter = 0
        self._decimation = 4
    
    def get_odom_info(self):
        return self.pos_IB, self.q_IB, self.lin_vel_I, self.ang_vel_I

    def _compute_observation(self, command):
        """
        Compute the observation vector for the policy

        Argument:
        command {np.ndarray} -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.

        """
        self.lin_vel_I = self.robot.get_linear_velocity()
        self.ang_vel_I = self.robot.get_angular_velocity()
        self.pos_IB, self.q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(self.q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, self.lin_vel_I) # R_BI @ lin_vel_I
        ang_vel_b = np.matmul(R_BI, self.ang_vel_I) # R_BI @ ang_vel_I
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0])) 

        obs = np.zeros(48)
        # Base lin vel
        obs[:3] = self._base_vel_lin_scale * lin_vel_b
        # Base ang vel
        obs[3:6] = self._base_vel_ang_scale * ang_vel_b
        # Gravity
        obs[6:9] = gravity_b
        # Command
        obs[9] = self._base_vel_lin_scale * command[0]
        obs[10] = self._base_vel_lin_scale * command[1]
        obs[11] = self._base_vel_ang_scale * command[2]
        # Joint states
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        obs[12:24] = current_joint_pos - self._default_joint_pos
        obs[24:36] = current_joint_vel
        # Previous Action
        obs[36:48] = self._previous_action

        return obs

    def advance(self, dt, command):
        """
        Compute the desired torques and apply them to the articulation

        Argument:
        dt {float} -- Timestep update in the world.
        command {np.ndarray} -- the robot command (v_x, v_y, w_z)

        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            with torch.no_grad():
                obs = torch.from_numpy(obs).view(1, -1).float()
                self.action = self._policy(obs).detach().view(-1).numpy()
            self._previous_action = self.action.copy()

        # print(f'action: {self._default_joint_pos + (self.action * self._action_scale)}')
        action = ArticulationAction(joint_positions=self._default_joint_pos + (self.action * self._action_scale))
        self.robot.apply_action(action)

        self._policy_counter += 1

    def initialize(self, physics_sim_view=None) -> None:
        """
        Initialize robot the articulation interface, set up drive mode
        """
        self.robot.initialize(physics_sim_view=physics_sim_view)
        self.robot.get_articulation_controller().set_effort_modes("force")
        self.robot.get_articulation_controller().switch_control_mode("position")
        self.robot._articulation_view.set_gains(np.zeros(12) + 25, np.zeros(12) + 0.5)

    def post_reset(self) -> None:
        """
        Post reset articulation
        """
        self.robot.post_reset()