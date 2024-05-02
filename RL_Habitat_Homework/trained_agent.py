import os
import random
from typing import Dict, List

import imageio
import numpy as np
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
from omegaconf import OmegaConf, DictConfig


# A function to build the evaluation config for the trained agent
def build_pretrained_config(data_path: str):
    config = get_config("pointnav/ppo_pointnav.yaml")  # Extract config from yaml
    # Change for evaluation
    OmegaConf.set_readonly(config, False)
    config.habitat_baselines.eval_ckpt_path_dir="data/checkpoints/gibson.pth"  # Choose checkpoint
    config.habitat_baselines.num_updates = -1
    config.habitat_baselines.num_environments = 1
    config.habitat_baselines.verbose = False
    config.habitat.dataset.data_path = data_path
    OmegaConf.set_readonly(config, True)

    return config


# A function to build a vectorized environment
def build_env(config: DictConfig, multiprocess=True):
    if not multiprocess:
        import os
        os.environ['HABITAT_ENV_DEBUG'] = '1'
    return construct_envs(
        config=config,
        workers_ignore_signals=False,
        enforce_scenes_greater_eq_environments=True,
    )


# A function to load the pretrained agent
def build_agent(config: DictConfig, env: VectorEnv, device: torch.device):
    ppo_cfg = config.habitat_baselines.rl.ppo  # Extract config for PPO

    policy = baseline_registry.get_policy(
        config.habitat_baselines.rl.policy.name
    )
    # TODO: Please enter your code here to replace ...
    # HINT: You can refer to the doc of VectorEnv at https://aihabitat.org/docs/habitat-lab/habitat.core.vector_env.VectorEnv.html
    observation_space = env.observation_spaces[0]
    policy_action_space = env.action_spaces[0]
    orig_policy_action_space = env.orig_action_spaces[0]

    actor_critic = policy.from_config(  # Build the actor-critic
        config,
        observation_space,
        policy_action_space,
        orig_action_space=orig_policy_action_space,
    )
    actor_critic.to(device)

    agent = PPO.from_config(  # Build the PPO agent
        actor_critic=actor_critic,
        config=ppo_cfg,
    )

    ckpt_dict = torch.load(config.habitat_baselines.eval_ckpt_path_dir, map_location="cpu")  # Load the checkpoint
    agent.load_state_dict(ckpt_dict["state_dict"])

    actor_critic.eval()
    agent.eval()

    return actor_critic, agent


sample_config = build_pretrained_config("data/datasets/pointnav/gibson/v1/val/val_cantwell.json.gz")
sample_env = build_env(config=sample_config)
sample_device = torch.device("cpu")
sample_actor_critic, sample_agent = build_agent(config=sample_config, env=sample_env, device=sample_device)


# A function to build auxiliary variables for the policy
def build_variables(config: DictConfig, actor_critic: nn.Module, device: torch.device):
    test_recurrent_hidden_states = torch.zeros(  # Hidden recurrent state
        config.habitat_baselines.num_environments,
        actor_critic.num_recurrent_layers,
        config.habitat_baselines.rl.ppo.hidden_size,
        device=device
    )
    prev_actions = torch.zeros(  # Previous action
        config.habitat_baselines.num_environments,
        1,
        device=device,
        dtype=torch.long,
    )
    not_done_masks = torch.zeros(
        config.habitat_baselines.num_environments,
        1,
        device=device,
        dtype=torch.bool,
    )

    return test_recurrent_hidden_states, prev_actions, not_done_masks


sample_test_recurrent_hidden_states, sample_prev_actions, sample_not_done_masks = build_variables(
    config=sample_config,
    actor_critic=sample_actor_critic,
    device=sample_device
)


# A function to map observations to actions using the pretained agent
def step_agent(actor_critic: nn.Module, batch: Dict[str, torch.Tensor], test_recurrent_hidden_states: torch.Tensor,
               prev_actions: torch.Tensor, not_done_masks: torch.Tensor):
    # TODO: Please enter your code here to replace ...
    # HINT: You can refer to the resnet policy at https://github.com/facebookresearch/habitat-lab/blob/v0.2.3/habitat-baselines/habitat_baselines/rl/ddppo/policy/resnet_policy.py
    with inference_mode():
        (
            _,
            actions,
            _,
            test_recurrent_hidden_states,
        ) = actor_critic.act(
            batch,  # Observations
            test_recurrent_hidden_states,  # Recurrent hidden states
            prev_actions,  # Previous actions
            not_done_masks,  # Masks indicating whether episodes have ended
            deterministic=False,  # Non-deterministic actions for exploration
        )

        prev_actions.copy_(actions)  # type: ignore

    return actions, test_recurrent_hidden_states


# A function to excecute simulation within the environment
def step_env(env: VectorEnv, actions: torch.Tensor):
    # TODO: Please enter your code here to replace ...
    # HINT: You can refer to the doc of VectorEnv at https://aihabitat.org/docs/habitat-lab/habitat.core.vector_env.VectorEnv.html
    step_data = actions.tolist()[0]
    outputs = env.step(step_data)

    observations, rewards_l, dones, infos = [
        list(x) for x in zip(*outputs)
    ]

    return observations, rewards_l, dones, infos


# A utility function to post-process the results
def post_process(observations: List[Dict], dones: List[bool], device: torch.device):
    batch = batch_obs(  # type: ignore
        observations,
        device=device,
    )

    not_done_masks = torch.tensor(
        [[not done] for done in dones],
        dtype=torch.bool,
        device=device,
    )

    return batch, not_done_masks


# A utility function to collect rewards in an episode
def collect_rewards(rewards_l: List[float], ep_rewards: List[float]):
    rewards = torch.tensor(
        rewards_l, dtype=torch.float, device="cpu"
    ).unsqueeze(1)
    ep_rewards.append(rewards[0].item())


# A utility function to collect frames in an episode
def collect_frames(batch: Dict[str, torch.Tensor], infos: List[Dict], not_done_masks: torch.Tensor,
                   ep_frames: List[np.ndarray]):
    frame = observations_to_image(
        {k: v[0] for k, v in batch.items()}, infos[0]
    )
    if not not_done_masks[0].item():
        # The last frame corresponds to the first frame of the next episode
        # but the info is correct. So we use a black frame
        frame = observations_to_image(
            {k: v[0] * 0.0 for k, v in batch.items()}, infos[0]
        )
    frame = overlay_frame(frame, infos[0])
    ep_frames.append(frame)


# A utility function to generate video from collected frames
def generate_video(ep_frames: List[np.ndarray], video_name: str, output_dir: str = "output_video", fps: int = 10):
    output_dir = output_dir.lower().replace(" ", "_")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    video_name = video_name.lower().replace(" ", "_")

    writer = imageio.get_writer(
        os.path.join(output_dir, video_name),
        fps=fps
    )

    frames_iter = tqdm.tqdm(ep_frames)
    for fm in frames_iter:
        writer.append_data(fm)
    writer.close()


def loop_env(config: DictConfig, scene_name: str, agent_name: str, add_drift: bool = False, drift_func = None):
    # Set the randomness requried later
    random.seed(config.habitat.seed)
    np.random.seed(config.habitat.seed)
    torch.manual_seed(config.habitat.seed)

    # Choose device for cuda or cpu
    device = torch.device("cuda", config.habitat_baselines.torch_gpu_id) if torch.cuda.is_available() else torch.device("cpu")

    # Build the vectorized environment
    env = build_env(config=config)

    # Build the actor-critic and agent from a checkpoint
    actor_critic, agent = build_agent(config=config, env=env, device=device)

    # Build auxiliary variables
    test_recurrent_hidden_states, prev_actions, not_done_masks = build_variables(
        config=config,
        actor_critic=actor_critic,
        device=device
    )

    observations = env.reset()  # Reset the environment, e.g., move the agent back to its start location
    batch = batch_obs(observations, device=device)

    rng = np.random.default_rng()
    sign, scale = None, None

    n_step = 0
    episodes_stats = {}
    ep_rewards, ep_frames = [], []
    while len(episodes_stats) < env.number_of_episodes[0]:
        ep_name = env.current_episodes()[0].episode_id
        actions, test_recurrent_hidden_states = step_agent(  # Map observations to actions
            actor_critic=actor_critic,
            batch=batch,
            test_recurrent_hidden_states=test_recurrent_hidden_states,
            prev_actions=prev_actions,
            not_done_masks=not_done_masks,
        )

        observations, rewards_l, dones, infos = step_env(  # One step forward of simulation in the environment
            env=env,
            actions=actions,
        )
        if add_drift:
            if sign is None:
                sign = rng.choice((-1, 1))  # Direction for drift
                scale = 0.05  # Scale the drift w.r.t. steps
            drift_func(
                observations=observations,
                n_step=n_step,
                sign=sign,
                scale=scale
            )

        batch, not_done_masks = post_process(  # Post-process the results
            observations=observations,
            dones=dones,
            device=device,
        )

        collect_rewards(  # Collect rewards
            rewards_l=rewards_l,
            ep_rewards=ep_rewards,
        )

        collect_frames(  # Collect frames
            batch=batch,
            infos=infos,
            not_done_masks=not_done_masks,
            ep_frames=ep_frames
        )

        n_step += 1
        if not not_done_masks[0].item():  # Episode ended
            last_infos = infos.copy()

            episodes_stats[ep_name] = {
                'success': last_infos[0]['success'],
                'spl': last_infos[0]['spl'],
                'return': sum(ep_rewards),
            }

            # Generate video
            generate_video(
                ep_frames=ep_frames,
                video_name=f"ep={ep_name}_success={last_infos[0]['success']}_spl={last_infos[0]['spl']}.mp4",
                output_dir=f"output_video/{scene_name}/{agent_name}"
            )

            # Clean
            n_step = 0
            ep_rewards, ep_frames = [], []

            # Build auxiliary variables
            test_recurrent_hidden_states, prev_actions, not_done_masks = build_variables(
                config=config,
                actor_critic=actor_critic,
                device=device
            )

    success_l, spl_l, return_l = [], [], []
    for ep_stat in episodes_stats.values():
        success_l.append(ep_stat['success'])
        spl_l.append(ep_stat['spl'])
        return_l.append(ep_stat['return'])

    avg_success = sum(success_l) / len(success_l)
    avg_spl = sum(spl_l) / len(spl_l)
    avg_return = sum(return_l) / len(return_l)

    print(f"In {scene_name}, {agent_name}")
    print(f"\t Average success rate: {avg_success}")
    print(f"\t Average SPL: {avg_spl}")
    print(f"\t Average return: {avg_return}")

    return {
        'success': avg_success,
        'spl': avg_spl,
        'return': avg_return
    }


stats_pretrained_cantwell = loop_env(config=sample_config, scene_name='Cantwell', agent_name='Pretrained Agent')