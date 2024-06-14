# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run an environment with zero action agent."""

"""Launch Isaac Sim Simulator first."""

import argparse
from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Zero agent for Isaac Lab environments.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""
from base_env_setup import *
from omni.isaac.lab.envs import ManagerBasedRLEnv

def main():
    """Zero actions agent with Isaac Lab environment."""
    env_cfg = CubeEnvCfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)

    action = torch.zeros((env.num_envs, 6), device=env.device)

    # simulate physics
    count = 0
    obs, _ = env.reset()

    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            if count % 300 == 0:
                count = 0
                obs, _ = env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")

            # step env
            if count < 100:
                action[:] = torch.tensor((0, 0, 0, 0, 0, 0), device=env.device).expand_as(action)
            elif count < 150:
                action[:] = torch.tensor((0, 0, 0, 0, 0, 1), device=env.device).expand_as(action)
            elif count < 200:
                action[:] = torch.tensor((0, 0, 0, 0, 0, -1), device=env.device).expand_as(action)
            else:
                action[:] = torch.tensor((1, 0, 0, 0, 0, 0), device=env.device).expand_as(action)

            obs, rew, resets_terminated, res_truncated, extras = env.step(action)
            # print(f"obs: {obs}")
            # print(f"rew: {rew}")
            # print(f"resets terminated: {resets_terminated}")
            # print(f"res truncated: {res_truncated}")
            # print(f"extras: {extras}")

            # update counter
            count += 1

    # close the environment
    env.close()
if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
