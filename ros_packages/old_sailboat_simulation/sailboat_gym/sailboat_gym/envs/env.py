import gymnasium as gym
import numpy as np
from typing import Any, Tuple, Callable, Type, TypedDict

from ..types import GymObservation, GymAction, Observation, Action, ResetInfo
from ..utils import ProfilingMeta
from ..abstracts import AbcRender


class Metadata(TypedDict):
    render_modes: list[str]
    render_fps: float


class SailboatEnv(gym.Env, metaclass=ProfilingMeta):
    NB_STEPS_PER_SECONDS: int  # Hz

    render_mode: str
    metadata: Metadata

    action_space = GymAction
    observation_space = GymObservation

    def reset(self, **kwargs) -> Tuple[Observation, ResetInfo]:
        return super().reset(**kwargs)

    def step(self, action: Action) -> Tuple[Observation, float, bool, Any]:
        return super().step(action)

    def render(self, draw_extra_fct: Callable[[AbcRender, np.ndarray, Observation], None] = None) -> np.ndarray:
        return super().render()

    def close(self) -> None:
        return super().close()
