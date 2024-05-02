import seaborn as sns
import matplotlib
import random
from PIL import Image
from matplotlib import pyplot as plt
import numpy as np

import os
import random
from typing import Dict, List

import imageio
import matplotlib.pyplot as plt
import numpy as np
from omegaconf import OmegaConf
from omegaconf.dictconfig import DictConfig

import torch
import torch.nn as nn
import tqdm

import habitat_sim
from habitat import VectorEnv
from habitat.utils.visualizations.utils import observations_to_image
from habitat_baselines.common.baseline_registry import baseline_registry
from habitat_baselines.config.default import get_config
from habitat_baselines.utils.common import (
    batch_obs,
    inference_mode,
)
from habitat_baselines.rl.ppo import PPO

from habitat.utils.render_wrapper import overlay_frame
from habitat_baselines.common.construct_vector_env import construct_envs

matplotlib.rcParams["figure.dpi"] = 100
sns.set_style("whitegrid")

sensor_settings = {
    "height": 256, "width": 256,  # Spatial resolution of observations
    "sensor_height": 1.0,  # Height of sensors in meters, relative to the agent
}

sim_settings = {
    "default_agent": 0,  # Index of the default agent
    "scene_id": "data/scene_datasets/gibson/Cantwell.glb",  # Scene file, episode 0 in val split of Gibson
    "enable_physics": False,  # kinematics only
    "seed": 42  # used in the random navigation
}


def main():
    sensor_specs = config_sensors()
    agent_cfg = agent_config(sensor_specs)
    sim_cfg = sim_backend()

    cfg = habitat_sim.Configuration(sim_cfg, [agent_cfg])
    sim = habitat_sim.Simulator(cfg)
    random.seed(sim_settings["seed"])   # Randomness is needed when choosing the actions
    sim.seed(sim_settings["seed"])

    set_agent_state(sim)
    dynmaic_test(sim)


def config_sensors():
    # Create a RGB sensor configuration
    rgb_sensor_spec = habitat_sim.CameraSensorSpec()
    rgb_sensor_spec.uuid = "color_sensor"
    rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgb_sensor_spec.resolution = [sensor_settings["height"], sensor_settings["width"]]
    rgb_sensor_spec.position = [0.0, sensor_settings["sensor_height"], 0.0]
    rgb_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

    # Create a depth sensor configuration
    depth_sensor_spec = habitat_sim.CameraSensorSpec()
    depth_sensor_spec.uuid = "depth_sensor"
    depth_sensor_spec.sensor_type = habitat_sim.SensorType.DEPTH
    depth_sensor_spec.resolution = [sensor_settings["height"], sensor_settings["width"]]
    depth_sensor_spec.position = [0.0, sensor_settings["sensor_height"], 0.0]
    depth_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

    return [rgb_sensor_spec, depth_sensor_spec]


def agent_config(sensor_specs):
    agent_settings = {
        "action_space": {
            "move_forward": 0.25, "move_backward": 0.25,  # Distance to cover in a move action in meters
            "turn_left": 30.0, "turn_right": 30,  # Angles to cover in a turn action in degrees
        }
    }

    # Create an agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.action_space = {
        k: habitat_sim.agent.ActionSpec(
            k, habitat_sim.agent.ActuationSpec(amount=v)
        ) for k, v in agent_settings["action_space"].items()
    }
    agent_cfg.sensor_specifications = sensor_specs

    return agent_cfg


def sim_backend():
    # Create a simulator backend configuration
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = sim_settings["scene_id"]
    sim_cfg.enable_physics = sim_settings["enable_physics"]

    return sim_cfg


# A utility function for displaying observations
def display_obs(rgb_obs: np.ndarray, depth_obs: np.ndarray):
    img_arr, title_arr = [], []

    rgb_img = Image.fromarray(rgb_obs, mode="RGBA")
    img_arr.append(rgb_img)
    title_arr.append("rgb")

    depth_img = Image.fromarray((depth_obs / 10 * 255).astype(np.uint8), mode="L")
    img_arr.append(depth_img)
    title_arr.append("depth")

    plt.figure(figsize=(12, 8))
    for i, (img, title) in enumerate(zip(img_arr, title_arr)):
        ax = plt.subplot(1, 2, i + 1)
        ax.axis("off")
        ax.set_title(title)
        plt.imshow(img)
    plt.show(block=False)


def set_agent_state(sim):
    # Set agent state
    agent = sim.initialize_agent(sim_settings["default_agent"])  # Get our default agent
    agent_state = habitat_sim.AgentState()
    agent_state.position = np.array([-4.69643, 0.15825, -2.90618])  # Position in world coordinate
    agent.set_state(agent_state)

    # Get agent state
    agent_state = agent.get_state()
    print(f"Agent state: position ({agent_state.position}), rotation ({agent_state.rotation})")


def dynmaic_test(sim):
    key_bindings = {
        'T': 'move_forward',
        'G': 'move_backward',
        'F': 'turn_left',
        'H': 'turn_right',
    }
    while True:
        key_pressed = input('Use TGFH to control the agent now: ')
        key_pressed = key_pressed.upper()
        if key_pressed == 'O':
            print("Bye...")
            break
        elif key_pressed in key_bindings.keys():
            action = key_bindings[key_pressed]
            print(f"Choose to {action}")
            # TODO: Please enter your code here to replace ...
            # HINT: You can refer to the doc of Env at https://aihabitat.org/docs/habitat-lab/habitat.core.env.Env.html
            observations = sim.step(action)
            rgb_obs = observations.get("color_sensor")
            depth_obs = observations.get("depth_sensor")

            display_obs(rgb_obs, depth_obs)
        else:
            print(f"Invalid input {key_pressed}. Please only use TGFH!")


if __name__ == '__main__':
    main()
