from pathlib import Path
import random
from typing import Optional, Dict

from .monodepth2 import RGBtoDepthModel
import PIL.Image as pil

import torch
import cv2 as cv
import numpy as np
from gym.spaces import Box, Dict as SpaceDict, Discrete
from omegaconf import DictConfig

from .habitat_utils.agent import Agent
from .habitat_utils.simulator import Observations
from .habitat_utils.ppo_agents import PPOAgentConfig
from .habitat_utils.resnet_policy import PointNavResNetPolicy
from .habitat_utils.common import batch_obs


# CustomAgent is straight up a complete copy of PPOAgent (from
# habitat_baselines.agents.ppo_agets), but with the GOAL_SENSOR_UUID space
# commented out
class CustomAgent(Agent):
    def __init__(self, config: DictConfig, rgb_model_path: str) -> None:
        spaces = {
            # get_default_config().GOAL_SENSOR_UUID: Box(
            #     low=np.finfo(np.float32).min,
            #     high=np.finfo(np.float32).max,
            #     shape=(2,),
            #     dtype=np.float32,
            # )
        }

        if config.INPUT_TYPE in ["depth", "rgbd"]:
            spaces["depth"] = Box(
                low=0,
                high=1,
                shape=(config.RESOLUTION, config.RESOLUTION, 1),
                dtype=np.float32,
            )

        if config.INPUT_TYPE in ["rgb", "rgbd"]:
            spaces["rgb"] = Box(
                low=0,
                high=255,
                shape=(config.RESOLUTION, config.RESOLUTION, 3),
                dtype=np.uint8,
            )
        observation_spaces = SpaceDict(spaces)
        self.observation_spaces = observation_spaces

        action_spaces = Discrete(4)

        self.device = (
            torch.device("cuda:{}".format(config.PTH_GPU_ID))
            if torch.cuda.is_available()
            else torch.device("cpu")
        )
        self.hidden_size = config.HIDDEN_SIZE

        random.seed(config.RANDOM_SEED)
        torch.random.manual_seed(config.RANDOM_SEED)
        if torch.cuda.is_available():
            torch.backends.cudnn.deterministic = True  # type: ignore

        self.actor_critic = PointNavResNetPolicy(
            observation_space=observation_spaces,
            action_space=action_spaces,
            hidden_size=self.hidden_size,
            normalize_visual_inputs="rgb" in spaces,
        )
        self.actor_critic.to(self.device)

        if config.MODEL_PATH:
            ckpt = torch.load(config.MODEL_PATH, map_location=self.device)
            #  Filter only actor_critic weights
            self.actor_critic.load_state_dict(
                {  # type: ignore
                    k[len("actor_critic.") :]: v
                    for k, v in ckpt.items()
                    if "actor_critic" in k
                }
            )
        else:
            raise ValueError("Model path is required")

        self.test_recurrent_hidden_states: Optional[torch.Tensor] = None
        self.not_done_masks: Optional[torch.Tensor] = None
        self.prev_actions: Optional[torch.Tensor] = None
        self.rgb_model = RGBtoDepthModel(rgb_model_path)

    def reset(self) -> None:
        self.test_recurrent_hidden_states = torch.zeros(
            1,
            self.actor_critic.net.num_recurrent_layers,
            self.hidden_size,
            device=self.device,
        )
        self.not_done_masks = torch.zeros(
            1, 1, device=self.device, dtype=torch.bool
        )
        self.prev_actions = torch.zeros(
            1, 1, dtype=torch.long, device=self.device
        )

    def act(self, observations: Observations) -> Dict[str, int]:
        if observations.get("depth") is None:
            observations["depth"] = self.rgb_model.convert(observations["rgb"])

        # resize images
        for k, im_np in observations.items():
            # if k in self.obs_space.spaces:
            if k in self.observation_spaces.spaces:
                obs_shape = self.observation_spaces[k].shape

                if im_np.shape == obs_shape:
                    # already the correct shape, don't need to resize
                    continue

                im_np = cv.resize(im_np, obs_shape[:2])
                if len(im_np.shape) == 2:
                    # everything should be 3 channel
                    im_np = np.expand_dims(im_np, axis=-1)
                observations[k] = im_np
        batch = batch_obs([observations], device=self.device)

        with torch.no_grad():
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
            #  Make masks not done till reset (end of episode) will be called
            self.not_done_masks.fill_(True)
            self.prev_actions.copy_(actions)  # type: ignore
        return {"action": actions[0][0].item()}