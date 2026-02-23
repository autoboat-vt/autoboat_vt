from gymnasium import spaces
from typing import TypedDict
import numpy as np


class Observation(TypedDict):
    p_boat: np.ndarray[3]
    dt_p_boat: np.ndarray[3]
    theta_boat: np.ndarray[3]
    dt_theta_boat: np.ndarray[3]
    theta_rudder: np.ndarray[1]
    dt_theta_rudder: np.ndarray[1]
    theta_sail: np.ndarray[1]
    dt_theta_sail: np.ndarray[1]
    wind: np.ndarray[2]
    water: np.ndarray[2]


class Action(TypedDict):
    theta_rudder: np.ndarray[1]
    theta_sail: np.ndarray[1]


class ResetInfo(TypedDict):
    map_bounds: np.ndarray[2, 3]  # min, max


GymObservation = spaces.Dict(
    {
        "p_boat": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
        "dt_p_boat": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
        "theta_boat": spaces.Box(low=-np.pi, high=np.pi, shape=(3,), dtype=np.float32),
        "dt_theta_boat": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
        "theta_rudder": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
        "dt_theta_rudder": spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32),
        "theta_sail": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
        "dt_theta_sail": spaces.Box(low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32),
        "wind": spaces.Box(low=-np.inf, high=np.inf, shape=(2,), dtype=np.float32),
        "water": spaces.Box(low=-np.inf, high=np.inf, shape=(2,), dtype=np.float32),
    }
)

GymAction = spaces.Dict(
    {
        "theta_rudder": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
        "theta_sail": spaces.Box(low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32),
    }
)
