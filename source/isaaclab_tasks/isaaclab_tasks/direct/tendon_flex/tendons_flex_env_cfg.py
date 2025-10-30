# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab_assets.robots.tendons import SHADOW_HAND_CFG
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg #, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

@configclass
class TendonsFlexEnvCfg(DirectRLEnvCfg):
    # env
    decimation = 2
    episode_length_s = 5.0
    # - spaces definition, 28 spatial tendon actuators on shadow hand
    action_space = 28
    observation_space = 7
    state_space = 0
    rew_scale_alive = 1.0
    rew_scale_terminated = -2.0
    rew_scale_pole_pos = -1.0
    rew_scale_cart_vel = -0.01
    rew_scale_pole_vel = -0.005

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    # robot(s)
    robot_cfg: ArticulationCfg = SHADOW_HAND_CFG.replace(prim_path="/World/envs/env_.*/shadow_hand")

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)