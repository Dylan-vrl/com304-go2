from habitat_baselines.config.default import get_config

from omegaconf import OmegaConf


def main():
    config = get_config("/home/david/Desktop/EPFL/comm_proj/habitat-lab/habitat-baselines/habitat_baselines/config/pointnav/ddppo_pointnav.yaml")

    print(OmegaConf.to_yaml(config))

if __name__ == '__main__':
    main()