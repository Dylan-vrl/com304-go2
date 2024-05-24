from pathlib import Path
import random
from typing import Optional, Dict

import torch
import cv2 as cv
import numpy as np
from gym.spaces import Box, Dict as SpaceDict, Discrete
from omegaconf import DictConfig

import habitat
from habitat.core.agent import Agent
from habitat.core.simulator import Observations
from habitat_baselines.agents.ppo_agents import PPOAgentConfig
from habitat_baselines.rl.ddppo.policy import PointNavResNetPolicy
from habitat_baselines.utils.common import batch_obs


# CustomAgent is straight up a complete copy of PPOAgent (from
# habitat_baselines.agents.ppo_agets), but with the GOAL_SENSOR_UUID space
# commented out
class CustomAgent(Agent):
    def __init__(self, config: DictConfig) -> None:
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
                    for k, v in ckpt["state_dict"].items()
                    if "actor_critic" in k
                }
            )

        else:
            habitat.logger.error(
                "Model checkpoint wasn't loaded, evaluating " "a random model."
            )

        self.test_recurrent_hidden_states: Optional[torch.Tensor] = None
        self.not_done_masks: Optional[torch.Tensor] = None
        self.prev_actions: Optional[torch.Tensor] = None

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


model_path = Path(__file__).parent / 'models/latest.pth'

agent_config = PPOAgentConfig()
agent_config.INPUT_TYPE = "rgbd"
agent_config.MODEL_PATH = model_path
agent_config.GOAL_SENSOR_UUID = "pointgoal"
agent_config.RESOLUTION = 128

# copied from config, ORDER MATTERS DO NOT EDIT
actions = ["stop", "move_forward", "turn_left", "turn_right"]

agent = CustomAgent(config=agent_config)
agent.reset()

for _ in range(64):
    observations = {
        "rgb": np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8),
        "depth": np.random.rand(256, 256, 1).astype(np.float32),
        # "pointgoal": np.random.rand(32).astype(np.float32)  # Assuming the pointgoal embedding is 32-dimensional
    }

    action = agent.act(observations)['action']
    print(f"action: {action} : {actions[action]}")
