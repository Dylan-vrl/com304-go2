from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.managers import SceneEntityCfg
import torch.nn.functional as F

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv
    from omni.isaac.sensor import Camera, ContactSensor
    from omni.isaac.lab.assets import Articulation, RigidObject, AssetBase


def l2_distance(env: ManagerBasedRLEnv, cfg1: SceneEntityCfg = SceneEntityCfg("cube"),
                cfg2: SceneEntityCfg = SceneEntityCfg("ball")) -> torch.Tensor:
    obj1: RigidObject = env.scene[cfg1.name]
    obj2: RigidObject = env.scene[cfg2.name]

    sub = obj1.data.root_pos_w[:] - obj2.data.root_pos_w[:]
    sub[:, 2] = 0  # set all z to 0 as it's not needed
    return torch.norm(sub, dim=1).view(env.num_envs, -1)  # Should be tensor of dim (nb_envs)


def contact_forces(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces")):
    sensor: ContactSensor = env.scene[sensor_cfg.name]
    return sensor.data.net_forces_w.view(env.num_envs, -1)


def is_close_to(env: ManagerBasedRLEnv,
                cfg1: SceneEntityCfg = SceneEntityCfg("cube"),
                cfg2: SceneEntityCfg = SceneEntityCfg("ball"),
                threshold=0.2):
    return (l2_distance(env, cfg1, cfg2) < threshold).view(
        env.num_envs).float()  # Should be bool tensor of dim (nb_envs)


def is_close_once(env: ManagerBasedRLEnv, func=is_close_to):
    is_close = func(env)
    ret = torch.logical_and(is_close, ~env.close_reward_given)
    env.close_reward_given = torch.logical_or(env.close_reward_given, is_close)
    return ret


def angle_diff(env: ManagerBasedRLEnv,
               cfg1: SceneEntityCfg = SceneEntityCfg("cube"),
               cfg2: SceneEntityCfg = SceneEntityCfg("ball"),
               epsilon=1e-8):
    obj1: RigidObject = env.scene[cfg1.name]
    obj2: RigidObject = env.scene[cfg2.name]

    vect_roots = obj2.data.root_pos_w[:] - obj1.data.root_pos_w[:]
    vect_roots[:, 2] = 0  # set all z to 0 as it's not needed

    # Compute the norm of vect_roots and add epsilon to avoid division by zero if norm is 0
    vect_roots_norm = vect_roots / (torch.norm(vect_roots, dim=1, keepdim=True).expand_as(vect_roots) + epsilon)

    # Define the x-axis in the same frame
    x_axis = torch.tensor((1, 0, 0), device=env.device).expand_as(vect_roots_norm)

    # Compute cosine similarity with x axis (in robot frame)
    cosine_sim = F.cosine_similarity(vect_roots_norm, x_axis)
    cosine_sim = torch.clamp(cosine_sim, -1.0, 1.0)  # clip to [-1, 1] to avoid NaNs in acos
    angle_in_radians = torch.acos(cosine_sim)  # Compute the angle in radians
    return torch.rad2deg(angle_in_radians)  # Compute angle in degrees


def is_angle_close(env: ManagerBasedRLEnv,
                   cfg1: SceneEntityCfg = SceneEntityCfg("cube"),
                   cfg2: SceneEntityCfg = SceneEntityCfg("ball"),
                   threshold=10):
    return (torch.abs(angle_diff(env, cfg1, cfg2)) < threshold).float()


def reached_target(env: ManagerBasedRLEnv, dist_threshold=0.2, angle_threshold=10):
    reached = torch.logical_and(is_close_to(env, threshold=dist_threshold),
                                is_angle_close(env, threshold=angle_threshold))
    # Apply mask to only give reward the first time
    ret = torch.logical_and(reached, ~env.target_reward_given)
    # Update the target reward given flags
    env.target_reward_given = torch.logical_or(env.target_reward_given, reached)
    return ret.view(env.num_envs)


def reset_env_params(env: ManagerBasedRLEnv, env_ids: torch.Tensor):
    env.close_reward_given = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    env.target_reward_given = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)


def got_illegal_contacts(env: ManagerBasedRLEnv, threshold=0.01):
    forces = contact_forces(env)
    forces[:, 2] = 0  # set z components to 0 (gravity)
    return (forces.norm(dim=1) > threshold).view(env.num_envs).float()

def joint_pos_out_of_manual_limit(
        env: ManagerBasedRLEnv,
        room_cfg: SceneEntityCfg = SceneEntityCfg("room"),
        asset_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
        bounds: tuple[float, float, float, float] = (-3.0, 3.0, -3.0, 3.0)
) -> torch.Tensor:
    """Terminate when the asset's joint positions are outside of the configured bounds.

    Note:
        This function is similar to :func:`joint_pos_out_of_limit` but allows the user to specify the bounds manually.
    """
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    room: AssetBase = env.scene[room_cfg.name]

    if asset_cfg.joint_ids is None:
        asset_cfg.joint_ids = slice(None)

    # compute any violations
    out_of_x_lower = asset.data.root_pos_w[:, 0] < env.scene.env_origins[:, 0] + bounds[0]
    out_of_x_higher = asset.data.root_pos_w[:, 0] > env.scene.env_origins[:, 0] + bounds[1]
    out_of_y_lower = asset.data.root_pos_w[:, 1] < env.scene.env_origins[:, 1] + bounds[2]
    out_of_y_higher = asset.data.root_pos_w[:, 1] > env.scene.env_origins[:, 1] + bounds[3]

    out_x = torch.logical_or(out_of_x_higher, out_of_x_lower)
    out_y = torch.logical_or(out_of_y_higher, out_of_y_lower)

    return torch.logical_or(out_x, out_y)