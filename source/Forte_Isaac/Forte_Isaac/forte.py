# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Franka Emika robots.

The following configurations are available:

* :obj:`FRANKA_PANDA_CFG`: Franka Emika Panda robot with Panda hand
* :obj:`FRANKA_PANDA_HIGH_PD_CFG`: Franka Emika Panda robot with Panda hand with stiffer PD control
* :obj:`FRANKA_ROBOTIQ_GRIPPER_CFG`: Franka robot with Robotiq_2f_85 gripper

Reference: https://github.com/frankaemika/franka_ros
"""


import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

FORTE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/simon/Repositories/Forte/Forte_Isaac/Forte_Isaac/forte.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_yaw": 0.0,
            "shoulder_pitch": -1.0472,
            "shoulder_roll": 0.0,
            "elbow_pitch": 1.74533,
            "lower_arm_roll": 0.0,
            "wrist_pitch": 0.0,
            "wrist_roll": 0.0
        },
    ),
    actuators={
        "forte_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_yaw", "shoulder_pitch", "shoulder_roll", "elbow_pitch", "lower_arm_roll"],
            effort_limit_sim=87.0,
            stiffness=80.0,
            damping=4.0,
        ),
        "forte_wrist": ImplicitActuatorCfg(
            joint_names_expr=["wrist_pitch", "wrist_roll"],
            effort_limit_sim=12.0,
            stiffness=80.0,
            damping=4.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Franka Emika Panda robot."""


FORTE_HIGH_PD_CFG = FORTE_CFG.copy()
FORTE_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
FORTE_HIGH_PD_CFG.actuators["forte_shoulder"].stiffness = 400.0
FORTE_HIGH_PD_CFG.actuators["forte_shoulder"].damping = 80.0
FORTE_HIGH_PD_CFG.actuators["forte_wrist"].stiffness = 400.0
FORTE_HIGH_PD_CFG.actuators["forte_wrist"].damping = 80.0
"""Configuration of Franka Emika Panda robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""