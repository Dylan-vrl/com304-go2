#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import abc
import time
from enum import Enum
from typing import (
    TYPE_CHECKING,
    Any,
    Dict,
    Union,
)

import attr
import numpy as np
from gym import Space

if TYPE_CHECKING:
    try:
        from torch import Tensor
    except ImportError:
        pass
    from omegaconf import DictConfig

VisualObservation = Union[np.ndarray, "Tensor"]


@attr.s(auto_attribs=True)
class ActionSpaceConfiguration(metaclass=abc.ABCMeta):
    config: "DictConfig"

    @abc.abstractmethod
    def get(self) -> Any:
        raise NotImplementedError


class SensorTypes(Enum):
    r"""Enumeration of types of sensors."""

    NULL = 0
    COLOR = 1
    DEPTH = 2
    NORMAL = 3
    SEMANTIC = 4
    PATH = 5
    POSITION = 6
    FORCE = 7
    TENSOR = 8
    TEXT = 9
    MEASUREMENT = 10
    HEADING = 11
    TACTILE = 12
    TOKEN_IDS = 13


class Sensor(metaclass=abc.ABCMeta):
    r"""Represents a sensor that provides data from the environment to agent.

    :data uuid: universally unique id.
    :data sensor_type: type of Sensor, use SensorTypes enum if your sensor
        comes under one of it's categories.
    :data observation_space: ``gym.Space`` object corresponding to observation
        of sensor.

    The user of this class needs to implement the get_observation method and
    the user is also required to set the below attributes:
    """

    uuid: str
    config: "DictConfig"
    sensor_type: SensorTypes
    observation_space: Space

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self.config = kwargs["config"] if "config" in kwargs else None
        if hasattr(self.config, "uuid"):
            # We allow any sensor config to override the uuid
            self.uuid = self.config.uuid
        else:
            self.uuid = self._get_uuid(*args, **kwargs)
        self.sensor_type = self._get_sensor_type(*args, **kwargs)
        self.observation_space = self._get_observation_space(*args, **kwargs)

    def _get_uuid(self, *args: Any, **kwargs: Any) -> str:
        raise NotImplementedError

    def _get_sensor_type(self, *args: Any, **kwargs: Any) -> SensorTypes:
        raise NotImplementedError

    def _get_observation_space(self, *args: Any, **kwargs: Any) -> Space:
        raise NotImplementedError

    @abc.abstractmethod
    def get_observation(self, *args: Any, **kwargs: Any) -> Any:
        r"""
        Returns:
            current observation for Sensor.
        """
        raise NotImplementedError


class Observations(Dict[str, Any]):
    r"""Dictionary containing sensor observations"""

    def __init__(
        self,
        sensors: Dict[str, Sensor],
        *args: Any,
        should_time: bool = False,
        **kwargs: Any,
    ) -> None:
        """Constructor

        :param sensors: list of sensors whose observations are fetched and
            packaged.
        """
        data = []
        for uuid, sensor in sensors.items():
            t_start = time.time()
            data.append((uuid, sensor.get_observation(*args, **kwargs)))

            if should_time:
                kwargs["task"].add_perf_timing(f"sensors.{uuid}", t_start)

        super().__init__(data)
