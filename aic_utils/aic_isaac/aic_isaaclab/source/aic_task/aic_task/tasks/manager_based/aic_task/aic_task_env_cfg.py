# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
import os
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp import JointPositionActionCfg
from isaaclab.envs.mdp import DifferentialInverseKinematicsActionCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.sensors import TiledCameraCfg
from isaaclab.devices import DevicesCfg
from isaaclab.devices.keyboard import Se3KeyboardCfg
from isaaclab.devices.spacemouse import Se3SpaceMouseCfg
from isaaclab.devices.gamepad import Se3GamepadCfg

from . import mdp
from .mdp.events import randomize_dome_light, randomize_board_and_parts

# Resolve asset directory relative to this file (portable across machines)
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
AIC_ASSET_DIR = os.path.join(_THIS_DIR, "Intrinsic_assets")
AIC_SCENE_DIR = AIC_ASSET_DIR
AIC_PARTS_DIR = os.path.join(AIC_ASSET_DIR, "assets")

EXTENSION_PATH = os.path.dirname(os.path.abspath(__file__))

##
# Scene definition
##


@configclass
class AICTaskSceneCfg(InteractiveSceneCfg):
    """Scene for aic task: UR5e robot, aic_scene, task_board."""

    # UR5e + gripper (fully defined here using local asset)
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(AIC_ASSET_DIR, "aic_unified_robot_cable_sdf.usd"),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                # disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=8,
            ),
            activate_contact_sensors=False,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-0.18, -0.122, 0),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos={
                "shoulder_pan_joint": 0.1597,
                "shoulder_lift_joint": -1.3542,
                "elbow_joint": -1.6648,
                "wrist_1_joint": -1.6933,
                "wrist_2_joint": 1.5710,
                "wrist_3_joint": 1.4110,
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
                effort_limit_sim=87.0,
                stiffness=2000.0,
                damping=100.0,
            ),
            # "gripper": ImplicitActuatorCfg(
            #     joint_names_expr=[
            #         "gripper_left_finger_joint",
            #         "gripper_right_finger_joint",
            #     ],
            #     effort_limit_sim=20.0,
            #     stiffness=800.0,
            #     damping=40.0,
            # ),
        },
    )

    # cable = ArticulationCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/cable",
    #     spawn=None,
    #     init_state=ArticulationCfg.InitialStateCfg(),
    #     actuators={},
    #     articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    #         solver_position_iteration_count=64,
    #         solver_velocity_iteration_count=32,
    #     ),
    # )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )

    # world
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    aic_scene = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/aic_scene",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(AIC_SCENE_DIR, "scene", "aic.usd"),
            # usd_path=f"/home/nvidia/Downloads/aic_world.usd",
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, -1.15),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    task_board = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/task_board",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(
                AIC_PARTS_DIR, "Task Board Base", "task_board_rigid.usd"
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.2837, 0.229, 0.0),
            # rot=(0.70686, -0.01851, 0.70686, 0.01851),
        ),
    )

    sc_port = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/sc_port",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(AIC_PARTS_DIR, "SC Port", "sc_port.usd"),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.2904, 0.1928, 0.005),
            rot=(0.73136, 0.0, 0.0, -0.682),
        ),
    )

    sc_port_2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/sc_port_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(AIC_PARTS_DIR, "SC Port", "sc_port.usd"),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.2913, 0.1507, 0.005),
            rot=(0.73136, 0.0, 0.0, -0.682),
        ),
    )

    nic_card = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/nic_card",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join(AIC_PARTS_DIR, "NIC Card", "nic_card.usd"),
            # scale=(0.009, 0.009, 0.009),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.25135, 0.25229, 0.0743),
        ),
    )

    # nic_card_mount = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/nic_card_mount",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path=os.path.join(AIC_PARTS_DIR, "NIC Card Mount", "nic_card_mount_visual.usd"),
    #         scale=(0.00001, 0.00001, 0.00001),
    #     ),
    #     init_state=AssetBaseCfg.InitialStateCfg(
    #         pos=(1.02, -0.010, 0.080),
    #         rot=(0.7073, 0.7073, 0.7073, -0.7073),
    #     ),
    # )

    def __post_init__(self):
        super().__post_init__()

        _cam_spawn = sim_utils.PinholeCameraCfg(
            focal_length=22.48,
            focus_distance=0.0,
            horizontal_aperture=20.955,
            vertical_aperture=18.627,
            clipping_range=(0.07, 20.0),
        )

        self.center_camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/aic_unified_robot/center_camera_optical/center_camera",
            spawn=_cam_spawn,
            height=224,
            width=224,
            data_types=["rgb"],
            offset=TiledCameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.0),
                rot=(1.0, 0.0, 0.0, 0.0),
                convention="ros",
            ),
        )
        self.left_camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/aic_unified_robot/left_camera_optical/left_camera",
            spawn=_cam_spawn,
            height=224,
            width=224,
            data_types=["rgb"],
            offset=TiledCameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.0),
                rot=(1.0, 0.0, 0.0, 0.0),
                convention="ros",
            ),
        )
        self.right_camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/aic_unified_robot/right_camera_optical/right_camera",
            spawn=_cam_spawn,
            height=224,
            width=224,
            data_types=["rgb"],
            offset=TiledCameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.0),
                rot=(1.0, 0.0, 0.0, 0.0),
                convention="ros",
            ),
        )


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    ee_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,
        resampling_time_range=(4.0, 4.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.55, 0.75),
            pos_y=(-0.10, 0.02),
            pos_z=(0.01, 0.15),
            roll=(0.0, 0.0),
            pitch=MISSING,  # depends on end-effector axis
            yaw=(-3.14, 3.14),
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action: ActionTerm = MISSING
    gripper_action: ActionTerm | None = None


@configclass
class EventCfg:
    """Configuration for events."""

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.05, 0.05),
            "velocity_range": (0.0, 0.0),
        },
    )

    # randomize_robot_pose = EventTerm(
    #     func=mdp.reset_root_state_uniform,
    #     mode="reset",
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #         "pose_range": {
    #             "x": (-0.1, 0.1),   # random offset around init_state
    #             "y": (-0.1, 0.1),
    #             "z": (0.0, 0.0),
    #         },
    #         "velocity_range": {},
    #     },
    # )

    randomize_light = EventTerm(
        func=randomize_dome_light,
        mode="reset",
        params={
            "intensity_range": (1500.0, 3500.0),
            "color_range": ((0.5, 0.5, 0.5), (1.0, 1.0, 1.0)),
        },
    )

    randomize_board_and_parts = EventTerm(
        func=randomize_board_and_parts,
        mode="reset",
        params={
            "board_scene_name": "task_board",
            "board_default_pos": (0.2837, 0.229, 0.0),
            "board_range": {"x": (-0.005, 0.005), "y": (-0.005, 0.005)},
            "parts": [
                {
                    "scene_name": "sc_port",
                    "offset": (0.0067, -0.0362, 0.005),
                    "pose_range": {"x": (-0.005, 0.02)},
                },
                {
                    "scene_name": "sc_port_2",
                    "offset": (0.0076, -0.0783, 0.005),
                    "pose_range": {"x": (-0.005, 0.02)},
                },
                {
                    "scene_name": "nic_card",
                    "offset": (-0.03235, 0.02329, 0.0743),
                    "pose_range": {"y": (0.0, 0.12)},
                    "snap_step": {"y": 0.04},
                },
            ],
        },
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP: robot state, ee pose, pose command."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy: joint state, ee pose, pose command."""

        # Robot state (joint space)
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.01, n_max=0.01)
        )
        # End-effector pose in env frame (pos xyz + quat wxyz = 7 dims)
        eef_pose = ObsTerm(
            func=mdp.body_pose_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names="wrist_3_link")},
            noise=Unoise(n_min=-0.001, n_max=0.001),
        )
        # Command (target ee pose)
        pose_command = ObsTerm(
            func=mdp.generated_commands, params={"command_name": "ee_pose"}
        )

        # Body forces
        body_forces = ObsTerm(
            func=mdp.body_incoming_wrench,
            scale=0.1,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    body_names=[
                        "base_link",
                        "shoulder_link",
                        "upper_arm_link",
                        "forearm_link",
                        "wrist_1_link",
                        "wrist_2_link",
                        "wrist_3_link",
                    ],
                )
            },
        )

        center_rgb = ObsTerm(
            func=mdp.image_features,
            params={
                "sensor_cfg": SceneEntityCfg("center_camera"),
                "data_type": "rgb",
                "model_name": "resnet18",
            },
        )
        left_rgb = ObsTerm(
            func=mdp.image_features,
            params={
                "sensor_cfg": SceneEntityCfg("left_camera"),
                "data_type": "rgb",
                "model_name": "resnet18",
            },
        )
        right_rgb = ObsTerm(
            func=mdp.image_features,
            params={
                "sensor_cfg": SceneEntityCfg("right_camera"),
                "data_type": "rgb",
                "model_name": "resnet18",
            },
        )

        # Last action
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # -- Position tracking (coarse): L2 penalty drives the EE toward the target --
    end_effector_position_tracking = RewTerm(
        func=mdp.position_command_error,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "command_name": "ee_pose",
        },
    )
    # -- Position tracking (fine): tanh kernel provides dense signal near target --
    end_effector_position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "std": 0.1,
            "command_name": "ee_pose",
        },
    )
    # -- Position tracking (exponential): sharp bonus at very close range for insertion --
    end_effector_position_tracking_exp = RewTerm(
        func=mdp.position_command_error_exp,
        weight=0.3,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "sigma": 0.05,
            "command_name": "ee_pose",
        },
    )

    # -- Orientation tracking (coarse): angular-distance penalty --
    end_effector_orientation_tracking = RewTerm(
        func=mdp.orientation_command_error,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "command_name": "ee_pose",
        },
    )
    # -- Orientation tracking (fine): tanh kernel for precise alignment --
    end_effector_orientation_tracking_fine_grained = RewTerm(
        func=mdp.orientation_command_error_tanh,
        weight=0.05,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "std": 0.25,
            "command_name": "ee_pose",
        },
    )

    # -- Sparse reaching bonus: +1 when EE is within 2 cm of the target --
    reaching_bonus = RewTerm(
        func=mdp.ee_reaching_bonus,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=MISSING),
            "threshold": 0.02,
            "command_name": "ee_pose",
        },
    )

    # -- Smoothness penalties --
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.0001)
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0001,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )
    joint_acc = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-1.0e-7,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )
    joint_torques = RewTerm(
        func=mdp.joint_torques_l2,
        weight=-1.0e-6,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    # -- Safety: penalize joints approaching their limits --
    joint_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


##
# Environment configuration
##


@configclass
class AICTaskEnvCfg(ManagerBasedRLEnvCfg):
    """AIC task env: UR5e robot and custom scene."""

    # Scene settings
    scene: AICTaskSceneCfg = AICTaskSceneCfg(num_envs=1, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self) -> None:
        super().__post_init__()

        # General settings
        self.decimation = 4
        self.sim.render_interval = self.decimation
        self.episode_length_s = 200.0
        self.sim.dt = 1.0 / 120.0
        # self.sim.gravity = (0.0, 0.0, 3)
        self.viewer.eye = (8.0, 0.0, 5.0)

        # Override reward/command body to UR end-effector
        ee_body = ["wrist_3_link"]
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = (
            ee_body
        )
        self.rewards.end_effector_position_tracking_fine_grained.params[
            "asset_cfg"
        ].body_names = ee_body
        self.rewards.end_effector_position_tracking_exp.params[
            "asset_cfg"
        ].body_names = ee_body
        self.rewards.end_effector_orientation_tracking.params[
            "asset_cfg"
        ].body_names = ee_body
        self.rewards.end_effector_orientation_tracking_fine_grained.params[
            "asset_cfg"
        ].body_names = ee_body
        self.rewards.reaching_bonus.params["asset_cfg"].body_names = ee_body

        # # Arm action: joint position control
        # self.actions.arm_action = JointPositionActionCfg(
        #     asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
        # )

        # Arm action: differential IK (for teleoperation)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            body_name="wrist_3_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="svd",
                ik_params={"k_val": 1.0, "min_singular_value": 1e-5},
            ),
            scale=0.05,
        )

        # Command generator: end-effector body and pitch (wrist_3_link, EE along x)
        self.commands.ee_pose.body_name = "wrist_3_link"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        # Teleop device configuration
        self.teleop_devices = DevicesCfg(
            devices={
                "keyboard": Se3KeyboardCfg(
                    pos_sensitivity=0.08,
                    rot_sensitivity=0.05,
                    gripper_term=False,
                    sim_device=self.sim.device,
                ),
                "gamepad": Se3GamepadCfg(
                    gripper_term=False,
                    sim_device=self.sim.device,
                ),
                "spacemouse": Se3SpaceMouseCfg(
                    gripper_term=False,
                    sim_device=self.sim.device,
                ),
            },
        )
