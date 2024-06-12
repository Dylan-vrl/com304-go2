from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm

from configs.action import CubeActionTermCfg
from configs.scene import MySceneCfg
import omni.isaac.lab.envs.mdp as mdp
from mdp.observations import *
from mdp.rewards import *


@configclass
class ActionsCfg:
    vel: CubeActionTermCfg = CubeActionTermCfg(asset_name="cube")


@configclass
class ObservationsCfg:
    @configclass
    class SimCfg(ObsGroup):
        """Observations for simulation and rewards"""
        forces = ObsTerm(func=contact_forces)
        l2_dist = ObsTerm(func=l2_distance)
        #vel = ObsTerm(func=lambda env: env.scene["cube"].data.root_lin_vel_w)

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group"""
        camera = ObsTerm(func=cam_data)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    sim: SimCfg = SimCfg()


@configclass
class EventCfg:
    reset_cube = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-1, 1), "y": (-1, 1), "yaw": (0, 0)},
            "velocity_range": {"x": (0, 0), "y": (0, 0), "z": (0, 0)},
            "asset_cfg": SceneEntityCfg("cube"),
        },
    )
    reset_ball = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-1.5, 1.5), "y": (-1.0, 1.0), "z": (0.6, 0.6)},
            "velocity_range": {"x": (0, 0), "y": (0, 0), "z": (0, 0)},
            "asset_cfg": SceneEntityCfg("ball"),
        },
    )
    reset_vars = EventTerm(func=reset_env_params, mode="reset")


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=-0.5)

    # (2) Failure (out of bounds , timeout)
    terminating = RewTerm(func=joint_pos_out_of_manual_limit, weight=-50.0)
    time_out = RewTerm(func=mdp.time_out, weight=-1.0)

    # (3) Rewards
    is_close = RewTerm(func=is_close_once, weight=40)  # Intermediary reward if close to the target
    target_reached = RewTerm(func=reached_target, weight=100)  # Reward when target is reached

    # (4) Negative rewards
    illegal_contacts = RewTerm(func=got_illegal_contacts, weight=-10)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Cart out of bounds
    dog_out_of_bounds = DoneTerm(
        func=joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("cube"),
                "room_cfg": SceneEntityCfg("room"),
                "bounds": (-3.0, 3.0, -3.0, 3.0)},
    )
    ball_out_of_bounds = DoneTerm(
        func=joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("ball"),
                "room_cfg": SceneEntityCfg("room"),
                "bounds": (-3.0, 3.0, -3.0, 3.0)},
    )
    reached_target = DoneTerm(func=reached_target, time_out=True)


@configclass
class CurriculumCfg:
    """Configuration for the curriculum."""
    pass


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""
    # no commands for this MDP
    null = mdp.NullCommandCfg()


@configclass
class CubeEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""
    # Scene settings
    scene: MySceneCfg = MySceneCfg(num_envs=2, env_spacing=10)
    # Basic settings
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    commands: CommandsCfg = CommandsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    # RL settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2  # run model at half the speed of the physics (60Hz)
        self.episode_length_s = 10

        # simulation settings
        self.sim.dt = 1 / 200.0  # run Physics at 300Hz (0.01 = 100Hz)

        # update sensor update periods
        # we tick all the sensors based on the smallest update period (physics update period)
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
        if self.scene.camera is not None:
            self.scene.camera.update_period = self.sim.dt

        self.close_reward_given = torch.zeros(self.scene.num_envs, dtype=torch.bool, device=self.sim.device)
        self.target_reward_given = torch.zeros(self.scene.num_envs, dtype=torch.bool, device=self.sim.device)
