from omni.isaac.lab.assets import RigidObject
from omni.isaac.lab.envs import ManagerBasedEnv
from omni.isaac.lab.managers import ActionTerm, ActionTermCfg
import torch

from omni.isaac.lab.utils import configclass


class CubeActionTerm(ActionTerm):
    """Simple action term that implements Velocity control for the cube asset.

    The action term is applied to the cube asset. It involves two steps:

    1. **Process the raw actions**: Typically, this includes any transformations of the raw actions
       that are required to map them to the desired space. This is called once per environment step.
    2. **Apply the processed actions**: This step applies the processed actions to the asset.
       It is called once per simulation step.

    Preprocess actions sets the yaw and z velocities to 0 and copies the rest of the actions as is.
    """

    _asset: RigidObject
    """The articulation asset on which the action term is applied."""

    def __init__(self, cfg: 'CubeActionTermCfg', env: ManagerBasedEnv):
        # call super constructor
        super().__init__(cfg, env)
        # create buffers
        self._raw_actions = torch.zeros(env.num_envs, 6, device=self.device)
        self._processed_actions = torch.zeros(env.num_envs, 6, device=self.device)
        self._vel_command = torch.zeros(self.num_envs, 6, device=self.device)

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        return self._raw_actions.shape[1]

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    """
    Operations
    """

    def process_actions(self, actions: torch.Tensor):
        # store the raw actions
        self._raw_actions[:] = actions

        # set z, pitch and roll velocities to 0
        self._processed_actions = self._raw_actions
        self._processed_actions[:, 2] = self._asset.data.root_lin_vel_w[:, 2]
        self._processed_actions[:, 3:5] = 0

    def apply_actions(self):
        self._asset.write_root_velocity_to_sim(self._processed_actions)


@configclass
class CubeActionTermCfg(ActionTermCfg):
    """Configuration for the cube action term."""

    class_type: type = CubeActionTerm
    """The class corresponding to the action term."""

