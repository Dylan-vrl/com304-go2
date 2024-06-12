from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv
    from omni.isaac.sensor import Camera, ContactSensor


def cam_data_vector(env: ManagerBasedRLEnv) -> torch.Tensor():
    cam: Camera = env.scene["camera"]
    cam_rgb: torch.Tensor = cam.data.output["rgb"][:, :, :, :3].flatten(start_dim=1)
    cam_depth: torch.Tensor = cam.data.output["distance_to_image_plane"].flatten(start_dim=1)

    return torch.cat((cam_rgb, cam_depth.clamp(0, 10)), dim=1)


def cam_data(env: ManagerBasedRLEnv) -> torch.Tensor():
    cam: Camera = env.scene["camera"]
    cam_rgb: torch.Tensor = cam.data.output["rgb"][:, :, :, :3]
    # Clamp depth to 10 and make same dim as rgb
    cam_depth: torch.Tensor = cam.data.output["distance_to_image_plane"].clamp(0, 10).unsqueeze(-1)

    return torch.cat((cam_rgb, cam_depth), dim=3)

