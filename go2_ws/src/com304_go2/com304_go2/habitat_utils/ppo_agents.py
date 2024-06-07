#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from dataclasses import dataclass
from typing import Tuple
from omegaconf import DictConfig, OmegaConf


@dataclass
class PPOAgentConfig:
    INPUT_TYPE: str = "rgb"
    MODEL_PATH: str = "data/checkpoints/gibson-rgb-best.pth"
    RESOLUTION: Tuple[int, int] = (256, 256)
    HIDDEN_SIZE: int = 512
    RANDOM_SEED: int = 7
    PTH_GPU_ID: int = 0
    GOAL_SENSOR_UUID: str = "pointgoal_with_gps_compass"


def get_default_config() -> DictConfig:
    return OmegaConf.create(PPOAgentConfig())  # type: ignore[call-overload]
