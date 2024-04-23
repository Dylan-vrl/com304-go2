import os
import habitat
from habitat_baselines.config.default import get_config as get_baseline_config
from habitat_baselines.run import execute_exp


def main():
    # Path to the Habitat-Lab config file
    config_dir = "../config"
    config_filename = "object_nav_ppo.yaml"

    # Load the baseline configuration and the environment configuration
    config = get_baseline_config(os.path.join(config_dir, config_filename))

    # Set additional configurations if necessary, for example:
    config.defrost()
    config.TASK_CONFIG.DATASET.DATA_PATH = '/path/to/your/dataset_pointnav.yaml'
    config.TASK_CONFIG.DATASET.SCENES_DIR = '/path/to/your/scenes_directory'
    config.freeze()

    # Start training
    execute_exp(config, "ppo")

if __name__ == "__main__":
    main()
