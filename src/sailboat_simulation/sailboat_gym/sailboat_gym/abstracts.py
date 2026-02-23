from __future__ import annotations
import numpy as np
from abc import ABCMeta, abstractmethod
from typing import List, Callable

from .types import Observation
from .utils import ProfilingMeta


class ABCProfilingMeta(ABCMeta, ProfilingMeta):
    pass


class AbcRender(metaclass=ABCProfilingMeta):
    @abstractmethod
    def get_render_mode(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def get_render_modes(self) -> List[str]:
        raise NotImplementedError

    @abstractmethod
    def setup(self, map_bounds: np.ndarray[2, 3]) -> None:
        raise NotImplementedError

    @abstractmethod
    def render(
        self, observation: Observation, draw_extra_fct: Callable[[AbcRender, np.ndarray, Observation], None] = None
    ) -> np.ndarray:
        raise NotImplementedError
