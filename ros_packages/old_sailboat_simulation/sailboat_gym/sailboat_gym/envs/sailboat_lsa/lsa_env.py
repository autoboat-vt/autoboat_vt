import numpy as np
from typing import Callable, Union

from ...abstracts import AbcRender
from ...types import Observation, Action
from ...utils import is_debugging_all
from ..env import SailboatEnv
from .lsa_sim import LSASim


class SailboatLSAEnv(SailboatEnv):
    NB_STEPS_PER_SECONDS = 10  # Hz

    def __init__(
        self,
        reward_fn: Callable[[Observation, Action, Observation], float] = lambda *_: 0,
        renderer: Union[AbcRender, None] = None,
        wind_generator_fn: Union[Callable[[int], np.ndarray], None] = None,
        water_generator_fn: Union[Callable[[int], np.ndarray], None] = None,
        video_speed: float = 1,
        keep_sim_alive: bool = False,
        name="default",
        map_scale=1,
        stop_condition_fn: Callable[[Observation, Action, Observation], bool] = lambda *_: False,
    ):
        """Sailboat LSA environment

        Args:
            reward_fn (Callable[[Observation, Action], float], optional): Use a custom reward function depending of your task. Defaults to lambda *_: 0.
            renderer (AbcRender, optional): Renderer instance to be used for rendering the environment, look at sailboat_gym/renderers folder for more information. Defaults to None.
            wind_generator_fn (Callable[[int], np.ndarray], optional): Function that returns a 2D vector representing the global wind during the simulation. Defaults to None.
            water_generator_fn (Callable[[int], np.ndarray], optional): Function that returns a 2D vector representing the global water current during the simulation. Defaults to None.
            video_speed (float, optional): Speed of the video recording. Defaults to 1.
            keep_sim_alive (bool, optional): Keep the simulation running even after the program exits. Defaults to False.
            name ([type], optional): Name of the simulation, required to run multiples environment on same machine.. Defaults to 'default'.
            map_scale (int, optional): Scale of the map, used to scale the map in the renderer. Defaults to 1.
        """
        super().__init__()

        # IMPORTANT: The following variables are required by the gymnasium API
        self.render_mode = renderer.get_render_mode() if renderer else None
        self.metadata = {
            "render_modes": renderer.get_render_modes() if renderer else [],
            "render_fps": float(video_speed * self.NB_STEPS_PER_SECONDS),
        }

        def direction_generator(std=1.0):
            direction = np.random.normal(0, std, 2)

            def generate_direction(step_idx):
                return direction

            return generate_direction

        self.name = name
        self.reward_fn = reward_fn
        self.stop_condition_fn = stop_condition_fn
        self.renderer = renderer
        self.obs = None
        self.wind_generator_fn = wind_generator_fn if wind_generator_fn else direction_generator()
        self.water_generator_fn = water_generator_fn if water_generator_fn else direction_generator(0.01)
        self.map_scale = map_scale
        self.keep_sim_alive = keep_sim_alive
        self.step_idx = 0
        self.sim = LSASim(self.name)

    def reset(self, seed=None, **kwargs):
        super().reset(seed=seed, **kwargs)
        if seed is not None:
            np.random.seed(seed)
        self.step_idx = 0

        wind = self.wind_generator_fn(self.step_idx)
        water = self.water_generator_fn(self.step_idx)

        self.obs, info = self.sim.reset(wind, water, self.NB_STEPS_PER_SECONDS)

        # setup the renderer, its needed to know the min/max position of the boat
        if self.renderer:
            self.renderer.setup(info["map_bounds"] * self.map_scale)

        if is_debugging_all():
            print("\nResetting environment:")
            print(f"  -> Wind: {wind}")
            print(f"  -> Water: {water}")
            print(f"  -> frequency: {self.NB_STEPS_PER_SECONDS} Hz")
            print(f"  <- Obs: {self.obs}")
            print(f"  <- Info: {info}")

        return self.obs, info

    def step(self, action: Action):
        assert self.obs is not None, "Please call reset before step"

        self.step_idx += 1

        wind = self.wind_generator_fn(self.step_idx)
        water = self.water_generator_fn(self.step_idx)

        next_obs, terminated, info = self.sim.step(wind, water, action)
        reward = self.reward_fn(self.obs, action, next_obs)
        truncated = self.stop_condition_fn(self.obs, action, next_obs)
        self.obs = next_obs

        if is_debugging_all():
            print("\nStepping environment:")
            print(f"  -> Wind: {wind}")
            print(f"  -> Water: {water}")
            print(f"  -> Action: {action}")
            print(f"  <- Obs: {self.obs}")
            print(f"  <- Reward: {reward}")
            print(f"  <- Terminated: {terminated}")
            print(f"  <- Info: {info}")

        return self.obs, reward, terminated, truncated, info

    def render(self):
        assert self.renderer, "No renderer"
        assert self.obs is not None, "Please call reset before render"
        return self.renderer.render(self.obs)

    def close(self):
        self.sim.close()
        self.obs = None

    def __del__(self):
        if not self.keep_sim_alive:
            self.sim.stop()
