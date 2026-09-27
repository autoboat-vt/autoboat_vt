from typing import List
import cv2
import numpy as np
from pydantic.utils import deep_update

from ..types import Observation
from ..abstracts import AbcRender
from ..utils import ProfilingMeta


BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (200, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 120, 0)
CYAN = (0, 255, 255)


def rgba(color, alpha, background=np.array([255, 255, 255])):
    return tuple((1 - alpha) * background + alpha * np.array(color))


def angle_to_vec(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def rotate_vector(vector: np.ndarray, angle: float):
    assert vector.shape == (2,)
    rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return rot @ vector


class RendererObservation(metaclass=ProfilingMeta):
    def __init__(self, obs: Observation):
        # heading angle
        self.theta_boat = obs["theta_boat"][2]
        self.dt_theta_boat = np.abs(obs["dt_theta_boat"][2]) * angle_to_vec(
            self.theta_boat + np.sign(obs["dt_theta_boat"][2]) * np.pi / 2
        )  # is either -90 or 90 degrees

        # position
        self.p_boat = np.array([obs["p_boat"][0], obs["p_boat"][1]])
        self.dt_p_boat = np.array([obs["dt_p_boat"][0], obs["dt_p_boat"][1]])

        # rudder
        self.theta_rudder = np.pi + self.theta_boat + obs["theta_rudder"][0]
        self.dt_rudder = np.abs(obs["dt_theta_rudder"][0]) * angle_to_vec(
            self.theta_rudder + np.sign(obs["dt_theta_rudder"][0]) * np.pi / 2
        )  # is either -90 or 90 degrees

        # sail
        self.theta_sail = np.pi + self.theta_boat + obs["theta_sail"][0]
        self.dt_sail = np.abs(obs["dt_theta_sail"][0]) * angle_to_vec(
            self.theta_sail + np.sign(obs["dt_theta_sail"][0]) * np.pi / 2
        )  # is either -90 or 90 degrees

        # wind
        self.wind = obs["wind"]

        # water
        self.water = obs["water"]


class CV2DRenderer(AbcRender):
    def __init__(self, size=512, padding=30, vector_scale=10, style={}):
        self.size = size
        self.padding = padding
        self.vector_scale = vector_scale
        self.map_bounds = None
        self.center = None

        self.style = {
            "background": WHITE,
            "border": {
                "color": rgba(BLACK, 0.2),
                "width": 1,
            },
            "boat": {
                "color": rgba(BLACK, 0.3),
                "spike_coef": 2,
                "size": 10,
                "phi": np.deg2rad(40),
                "dt_p": {
                    "color": GREEN,
                    "width": 1,
                },
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
                "center": {
                    "color": WHITE,
                    "radius": 2,
                },
            },
            "rudder": {
                "color": rgba(BLACK, 0.3),
                "width": 2,
                "height": 5,
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
            },
            "sail": {
                "color": rgba(BLACK, 0.7),
                "width": 2,
                "height": 15,
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
            },
            "wind": {
                "color": rgba(BLUE, 0.5),
                "width": 2,
            },
            "water": {
                "color": rgba(CYAN, 0.5),
                "width": 2,
            },
        }
        self.style = deep_update(self.style, style)

    def _create_empty_img(self):
        bg = np.array(self.style["background"])
        img = bg[None, None, :] + np.zeros((self.size, self.size, 3))
        return img.astype(np.uint8)

    def _scale_to_fit_in_img(self, x):
        return x / (self.map_bounds[1] - self.map_bounds[0]).max() * (self.size - 2 * self.padding)

    def _translate_and_scale_to_fit_in_map(self, x):
        return self._scale_to_fit_in_img(x - self.map_bounds[0]) + self.padding

    def _transform_obs_to_fit_in_img(self, obs: RendererObservation):
        # translate and scale positions
        obs.p_boat = self._translate_and_scale_to_fit_in_map(obs.p_boat)

        # scale vectors
        obs.dt_p_boat = self._scale_to_fit_in_img(obs.dt_p_boat)
        obs.dt_theta_boat = self._scale_to_fit_in_img(obs.dt_theta_boat)
        obs.dt_rudder = self._scale_to_fit_in_img(obs.dt_rudder)
        obs.dt_sail = self._scale_to_fit_in_img(obs.dt_sail)
        obs.wind = self._scale_to_fit_in_img(obs.wind)
        obs.water = self._scale_to_fit_in_img(obs.water)

    def _draw_borders(self, img: np.ndarray):
        borders = self._translate_and_scale_to_fit_in_map(self.map_bounds).astype(int)
        cv2.rectangle(
            img,
            tuple(borders[0]),
            tuple(borders[1]),
            self.style["border"]["color"],
            self.style["border"]["width"],
            lineType=cv2.LINE_AA,
        )

    def _draw_wind(self, img: np.ndarray, obs: RendererObservation):
        img_center = np.array([self.size, self.size]) / 2
        cv2.arrowedLine(
            img,
            tuple(img_center.astype(int)),
            tuple((img_center + obs.wind * self.vector_scale).astype(int)),
            self.style["wind"]["color"],
            self.style["wind"]["width"],
            tipLength=0.2,
            line_type=cv2.LINE_AA,
        )

    def _draw_water(self, img: np.ndarray, obs: RendererObservation):
        img_center = np.array([self.size, self.size]) / 2
        cv2.arrowedLine(
            img,
            tuple(img_center.astype(int)),
            tuple((img_center + obs.water * self.vector_scale).astype(int)),
            self.style["water"]["color"],
            self.style["water"]["width"],
            tipLength=0.2,
            line_type=cv2.LINE_AA,
        )

    def _draw_boat(self, img: np.ndarray, obs: RendererObservation):
        boat_size = self.style["boat"]["size"]
        phi = self.style["boat"]["phi"]
        spike_coeff = self.style["boat"]["spike_coef"]
        sailboat_pts = np.array(
            [
                [obs.p_boat + angle_to_vec(obs.theta_boat + phi) * boat_size],
                [obs.p_boat + angle_to_vec(obs.theta_boat + (np.pi - phi)) * boat_size],
                [obs.p_boat + angle_to_vec(obs.theta_boat + (np.pi + phi)) * boat_size],
                [obs.p_boat + angle_to_vec(obs.theta_boat - phi) * boat_size],
                [obs.p_boat + angle_to_vec(obs.theta_boat) * spike_coeff * boat_size],
            ],
            dtype=int,
        )
        cv2.fillConvexPoly(img, sailboat_pts, self.style["boat"]["color"], lineType=cv2.LINE_AA)

    def _draw_sail(self, img: np.ndarray, obs: RendererObservation):
        sail_height = self.style["sail"]["height"]
        sail_start = obs.p_boat
        sail_end = sail_start + angle_to_vec(obs.theta_sail) * sail_height
        cv2.line(
            img,
            tuple(sail_start.astype(int)),
            tuple(sail_end.astype(int)),
            self.style["sail"]["color"],
            self.style["sail"]["width"],
            lineType=cv2.LINE_AA,
        )

    def _draw_rudder(self, img: np.ndarray, obs: RendererObservation):
        rudder_height = self.style["rudder"]["height"]
        boat_phi = self.style["boat"]["phi"]
        boat_size = self.style["boat"]["size"]
        back_of_boat = obs.p_boat + angle_to_vec(np.pi + obs.theta_boat) * np.cos(boat_phi) * boat_size
        rudder_start = back_of_boat
        rudder_end = rudder_start + angle_to_vec(obs.theta_rudder) * rudder_height
        cv2.line(
            img,
            tuple(rudder_start.astype(int)),
            tuple(rudder_end.astype(int)),
            self.style["rudder"]["color"],
            self.style["rudder"]["width"],
            lineType=cv2.LINE_AA,
        )

    def _draw_boat_pos_velocity(self, img: np.ndarray, obs: RendererObservation):
        dt_p_boat_start = obs.p_boat
        dt_p_boat_end = dt_p_boat_start + obs.dt_p_boat * self.vector_scale

        try:
            cv2.arrowedLine(
                img,
                tuple(dt_p_boat_start.astype(int)),
                tuple(dt_p_boat_end.astype(int)),
                self.style["boat"]["dt_p"]["color"],
                self.style["boat"]["dt_p"]["width"],
                tipLength=0.2,
                line_type=cv2.LINE_AA,
            )
        except:
            print("WARNING: Failed to render boat pos velocity likely because of a bug in the sim")


    def _draw_boat_heading_velocity(self, img: np.ndarray, obs: RendererObservation):
        spike_coeff = self.style["boat"]["spike_coef"]
        boat_size = self.style["boat"]["size"]
        front_of_boat = obs.p_boat + angle_to_vec(obs.theta_boat) * spike_coeff * boat_size
        obs.dt_p_boat
        dt_theta_boat_start = front_of_boat
        dt_theta_boat_end = dt_theta_boat_start + obs.dt_theta_boat * self.vector_scale

        try:
            cv2.arrowedLine(
                img,
                tuple(dt_theta_boat_start.astype(int)),
                tuple(dt_theta_boat_end.astype(int)),
                self.style["boat"]["dt_theta"]["color"],
                self.style["boat"]["dt_theta"]["width"],
                tipLength=0.2,
                line_type=cv2.LINE_AA,
            )
        except:
            print("WARNING: Failed to render boat heading velocity likely because of a bug in the sim")

    def _draw_rudder_velocity(self, img: np.ndarray, obs: RendererObservation):
        boat_phi = self.style["boat"]["phi"]
        boat_size = self.style["boat"]["size"]
        rudder_height = self.style["rudder"]["height"]
        back_of_boat = obs.p_boat + angle_to_vec(np.pi + obs.theta_boat) * np.cos(boat_phi) * boat_size
        dt_rudder_start = back_of_boat + angle_to_vec(obs.theta_rudder) * rudder_height
        dt_rudder_end = dt_rudder_start + obs.dt_rudder

        try:
            cv2.arrowedLine(
                img,
                tuple(dt_rudder_start.astype(int)),
                tuple(dt_rudder_end.astype(int)),
                self.style["rudder"]["dt_theta"]["color"],
                self.style["rudder"]["dt_theta"]["width"],
                tipLength=0.2,
                line_type=cv2.LINE_AA,
            )
        except:
            print("WARNING: Failed to render rudder velocity likely because of a bug in the sim")


    def _draw_sail_velocity(self, img: np.ndarray, obs: RendererObservation):
        sail_height = self.style["sail"]["height"]
        dt_sail_start = obs.p_boat + angle_to_vec(obs.theta_sail) * sail_height
        dt_sail_end = dt_sail_start + obs.dt_sail

        try:
            cv2.arrowedLine(
                img,
                tuple(dt_sail_start.astype(int)),
                tuple(dt_sail_end.astype(int)),
                self.style["sail"]["dt_theta"]["color"],
                self.style["sail"]["dt_theta"]["width"],
                tipLength=0.2,
                line_type=cv2.LINE_AA,
            )
        except:
            print("WARNING: Failed to render sail velocity likely because of a bug in the sim")


    def _draw_boat_center(self, img: np.ndarray, obs: RendererObservation):
        cv2.circle(
            img, tuple(obs.p_boat.astype(int)), self.style["boat"]["center"]["radius"], self.style["boat"]["center"]["color"], -1
        )

    def get_render_mode(self) -> str:
        return "rgb_array"

    def get_render_modes(self) -> List[str]:
        return ["rgb_array"]

    def setup(self, map_bounds):
        map_bounds = np.array([[-75, -75, 0], [75, 75, 0]])
        self.map_bounds = map_bounds[:, 0:2]  # ignore z axis
        self.center = (self.map_bounds[0] + self.map_bounds[1]) / 2

    def render(self, observation, draw_extra_fct=None):
        assert self.map_bounds is not None and self.center is not None, "Please call setup() first."

        img = self._create_empty_img()

        # prepare observation
        obs = RendererObservation(observation)
        self._transform_obs_to_fit_in_img(obs)

        # draw extra stuff
        if draw_extra_fct is not None:
            draw_extra_fct(img, observation)

        # draw map
        self._draw_borders(img)
        self._draw_wind(img, obs)
        self._draw_water(img, obs)
        self._draw_boat(img, obs)
        self._draw_boat_heading_velocity(img, obs)
        self._draw_boat_pos_velocity(img, obs)
        self._draw_rudder(img, obs)
        self._draw_rudder_velocity(img, obs)
        self._draw_sail(img, obs)
        self._draw_sail_velocity(img, obs)
        self._draw_boat_center(img, obs)

        # flip vertically
        img = img[::-1, :, :]

        return img
