import os
from collections import OrderedDict
from typing import TYPE_CHECKING, List

import numpy as np
import torch
import torch.nn as nn
from gym import spaces
from omegaconf import OmegaConf
from PIL import Image

from .habitat_utils.common import (
    batch_obs,
    get_num_actions,
    inference_mode,
    is_continuous_action_space,
)
from .habitat_utils.ppo import PPO
from .habitat_utils.read_write import read_write
from .habitat_utils.resnet_policy import PointNavResNetPolicy

if TYPE_CHECKING:
    from omegaconf import DictConfig


def get_observation_space(
    agent_config: "DictConfig",
    select_keys: List[str] = ['rgb', 'depth'],
):
    obs_dict = {}
    for sensor_config in agent_config.sim_sensors.values():
        sensor_type = sensor_config.type
        if sensor_type == "HabitatSimRGBSensor":
            obs_dict["rgb"] = spaces.Box(
                low=0, 
                high=255, 
                shape=(
                    sensor_config.height,
                    sensor_config.width,
                    3,
                ), 
                dtype=np.uint8
            )
        elif sensor_type == "HabitatSimDepthSensor":
            if sensor_config.normalize_depth:
                min_depth_value = 0
                max_depth_value = 1
            else:
                min_depth_value = sensor_config.min_depth
                max_depth_value = sensor_config.max_depth

            obs_dict["depth"] = spaces.Box(
                low=min_depth_value,
                high=max_depth_value,
                shape=(sensor_config.height, sensor_config.width, 1),
                dtype=np.float32,
            )
        else:
            raise NotImplementedError(
                f"Only RGB and Depth sensors are supported. Got sensor type: {sensor_type}"
            )

    for k in list(obs_dict.keys()):
        if k not in select_keys:
            obs_dict.pop(k)

    return spaces.Dict(obs_dict)


def get_action_space(
    actions_config: "DictConfig"
):
    return spaces.Discrete(len(actions_config))
    

class HabitatController(nn.Module):
    def __init__(self, cfg_path, ckpt_path) -> None:
        super().__init__()
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        config = OmegaConf.load(cfg_path)
        with read_write(config):
            config.habitat_baselines.num_environments = 1
        self.config = config

        ckpt_dict = torch.load(ckpt_path, map_location=self.device)

        self._obs_space = get_observation_space(
            config.habitat.simulator.agents.main_agent
        )

        action_space = get_action_space(config.habitat.task.actions)
        self.policy_action_space = action_space
        self.orig_policy_action_space = action_space
        
        if is_continuous_action_space(action_space):
            # Assume NONE of the actions are discrete
            action_shape = (get_num_actions(action_space),)
            discrete_actions = False
        else:
            # For discrete pointnav
            action_shape = (1,)
            discrete_actions = True

        ppo_cfg = config.habitat_baselines.rl.ppo
        self._setup_actor_critic_agent(ppo_cfg)
        
        self.agent.load_state_dict(ckpt_dict)
        self.actor_critic = self.agent.actor_critic
        self.actor_critic.eval()

        self.test_recurrent_hidden_states = torch.zeros(
            self.config.habitat_baselines.num_environments,
            self.actor_critic.num_recurrent_layers,
            ppo_cfg.hidden_size,
            device=self.device,
        )
        self.prev_actions = torch.zeros(
            self.config.habitat_baselines.num_environments,
            *action_shape,
            device=self.device,
            dtype=torch.long if discrete_actions else torch.float,
        )
        self.not_done_masks = torch.zeros(
            self.config.habitat_baselines.num_environments,
            1,
            device=self.device,
            dtype=torch.bool,
        )


    def _setup_actor_critic_agent(self, ppo_cfg: "DictConfig") -> None:
        policy = PointNavResNetPolicy  # hardcode as we only have this one
        observation_space = self.obs_space
        # No obs transforms

        self.actor_critic = policy.from_config(
            self.config,
            observation_space,
            self.policy_action_space,
            orig_action_space=self.orig_policy_action_space,
        )
        self.obs_space = observation_space
        self.actor_critic.to(self.device)

        # Do not load pretrained weights here will that that later

        self.agent = PPO.from_config(
            self.actor_critic, ppo_cfg
        )

    @property
    def obs_space(self):
        return self._obs_space

    @obs_space.setter
    def obs_space(self, new_obs_space):
        self._obs_space = new_obs_space


    def act(self, observations: OrderedDict):
        batch = batch_obs(
            [observations],
            device=self.device,
        )
        # No obs transforms
        with inference_mode():
            (
                _,
                actions,
                _,
                self.test_recurrent_hidden_states,
            ) = self.actor_critic.act(
                batch,
                self.test_recurrent_hidden_states,
                self.prev_actions,
                self.not_done_masks,
                deterministic=False,
            )
            
            self.prev_actions.copy_(actions)

        if is_continuous_action_space(self.policy_action_space):
            # Clipping actions to the specified limits
            step_data = [
                np.clip(
                    a.numpy(),
                    self.policy_action_space.low,
                    self.policy_action_space.high,
                )
                for a in actions.cpu()
            ]
        else:
            step_data = [a.item() for a in actions.cpu()]

        if step_data[0] == 0:  # stop
            dones = [True]
            
            self.not_done_masks = torch.tensor(
                [[not done] for done in dones],
                dtype=torch.bool,
                device=self.device,
            )

        return step_data[0]
