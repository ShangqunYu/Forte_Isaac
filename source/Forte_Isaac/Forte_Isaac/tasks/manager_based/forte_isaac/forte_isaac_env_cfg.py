# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
import sys
import os

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.sensors import ContactSensorCfg

# from . import mdp
import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg
from isaaclab_tasks.manager_based.manipulation.pick_place.pickplace_forte_env_cfg import PickPlaceForteEnvCfg
from isaaclab_tasks.manager_based.manipulation.lift.config.forte.joint_pos_env_cfg import ForteCubeLiftEnvCfg

#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'lift')))

#import joint_pos_env_cfg

from Forte_Isaac.forte import FORTE_CFG

'''
camera_cfg = CameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/wrist_link/Realsense/RSD455",
    update_period=0.1,
    height=480,
    width=640,
    data_types=["rgb", "distance_to_image_plane"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
    ),
)'''

@configclass
class ForteReachEnvCfg(ReachEnvCfg):
    #camera = Camera(cfg=camera_cfg)
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to franka
        self.scene.robot = FORTE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["end_effector_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["end_effector_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["end_effector_link"]

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["shoulder_yaw", "shoulder_pitch", "shoulder_roll", "elbow_pitch", "lower_arm_roll", "wrist_pitch", "wrist_roll"], scale=0.5, use_default_offset=True
        )
        # override command generator body
        # end-effector is along z-direction
        self.commands.ee_pose.body_name = "end_effector_link"

        # Why not (-math.pi, math.pi)
        self.commands.ee_pose.ranges.pitch = (math.pi, math.pi)

@configclass
class ForteReachEnvCfg_PLAY(ForteReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False


@configclass
class FortePickPlaceEnvCfg(PickPlaceForteEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = FORTE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["shoulder_yaw", "shoulder_pitch", "shoulder_roll", "elbow_pitch", "lower_arm_roll", "wrist_pitch", "wrist_roll"], scale=0.5, use_default_offset=True
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=[".*_end_effector_joint"],
            open_command_expr={".*_end_effector_joint": 0.04},
            close_command_expr={".*_end_effector_joint": 0.0},
        )

@configclass
class FortePickPlaceEnvCfg_PLAY(FortePickPlaceEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False

@configclass
class ForteLiftEnvCfg(ForteCubeLiftEnvCfg):
    #camera = Camera(cfg=camera_cfg)
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = FORTE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

@configclass
class ForteLiftEnvCfg_PLAY(ForteLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False