# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import RigidObjectCfg
from isaaclab.assets import DeformableObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip
from isaaclab.assets import Articulation, ArticulationCfg
import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg

import omni.usd
import torch
torch.autograd.set_detect_anomaly(True)

omni.usd.get_context().open_stage("/home/namiko/work/Honda_isaac/IsaacLab/franka_allegro_fixedfinger_3.usd")

@configclass
class FrankaCubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        # self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot = ArticulationCfg(
        prim_path="/Root/franka_allegro",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/namiko/work/Honda_isaac/IsaacLab/franka_allegro_fixedfinger_3.usd",
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=12, solver_velocity_iteration_count=1
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "panda_joint1": 1.157,
                "panda_joint2": -1.066,
                "panda_joint3": -0.155,
                "panda_joint4": -2.239,
                "panda_joint5": 0.0, #-1.841,
                "panda_joint6": 1.003,
                "panda_joint7": 0.469,
                # "index_joint_0" : 0.0,
                # "middle_joint_0" : 0.0,
                # "ring_joint_0" : 0.0,
                # "thumb_joint_0" : 0.3,
                "index_joint_1" : 0.0,
                # "index_joint_2" : 0.0,
                # "index_joint_3" : 0.0,
                "middle_joint_1" : 0.0,
                # "middle_joint_2" : 0.0,
                # "middle_joint_3" : 0.0,
                "ring_joint_1" : 0.0,
                # "ring_joint_2" : 0.0,
                # "ring_joint_3" : 0.0,
                "thumb_joint_1" : 0.0,
                # "thumb_joint_2" : 0.0,
                # "thumb_joint_3" : 0.0,
            },
            pos=(0.0, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 0.1034),
        ),
        actuators={
            "panda_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint[1-4]"],
                effort_limit=87.0,
                velocity_limit=2.175,
                stiffness=800,
                damping=4.0,
            ),
            "panda_forearm": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint5"],
                effort_limit=12.0,
                velocity_limit=2.61,
                stiffness=80.0,
                damping=4.0,
            ),
            "panda_hand": ImplicitActuatorCfg(
                joint_names_expr=["panda_joint[6-7]"],
                effort_limit=200.0,
                velocity_limit=0.2,
                stiffness=2000,
                damping=1e2,
            ),
            "allegro_hand": ImplicitActuatorCfg(
                joint_names_expr=[".*_joint_.*"],
                effort_limit=200.0,
                velocity_limit=200,
                stiffness=200000,
                damping=4,
            ),
        },
    )

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*joint.*"], scale=1.0, use_default_offset=False
            # asset_name="robot", joint_names=["panda_joint[1-7]"], scale=0.5, use_default_offset=True
        )
        # self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
        #     asset_name="robot",
        #     # joint_names=[".*_joint_.*"],
        #     # open_command_expr={".*_joint_.*": 0.4},
        #     # close_command_expr={".*_joint_.*": 0.0},
        #     joint_names=["thumb_joint_3"],
        #     open_command_expr={"thumb_joint_3": 0.4},
        #     close_command_expr={"thumb_joint_3": 0.0},
        # )
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "middle_biotac_tip"

        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )
        # self.scene.object = DeformableObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Object",
        #     init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.5, 0, 0.05), rot=(0.707, 0, 0, 0.707)),
        #     spawn=UsdFileCfg(
        #         usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Objects/Teddy_Bear/teddy_bear.usd",
        #         scale=(0.01, 0.01, 0.01),
        #     ),
        # )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="/Root/franka_allegro/franka/panda_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="/Root/franka_allegro/allegro_hand/middle_biotac_tip",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.1034],
                    ),
                ),
            ],
        )


@configclass
class FrankaCubeLiftEnvCfg_PLAY(FrankaCubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
