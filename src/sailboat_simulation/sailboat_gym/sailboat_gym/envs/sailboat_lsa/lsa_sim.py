import docker
from docker.types import DeviceRequest
import zmq
import msgpack
import time
import threading
import numpy as np
import sys
import re
import os
from typing import TypedDict

from ...utils import ProfilingMeta, is_debugging, is_debugging_all, DurationProgress
from ...types import Action, Observation, ResetInfo


class AutoPauseIfInactive:
    def __init__(self, pause_fn, resume_fn) -> None:
        self.pause_fn = pause_fn
        self.resume_fn = resume_fn
        self.lock = threading.Lock()
        self.t = None

    def _on_pause(self):
        with self.lock:
            self.pause_fn()

    def __enter__(self):
        with self.lock:
            try:
                self.t.cancel()
            except Exception:
                pass
            self.resume_fn()

    def __exit__(self, exc_type, exc_value, exc_tb):
        self.t = threading.Timer(1, self._on_pause)
        self.t.start()


class Vector3(TypedDict):
    x: float
    y: float
    z: float


class Vector2(TypedDict):
    x: float
    y: float


class SimObservation(TypedDict):
    p_boat: Vector3
    dt_p_boat: Vector3
    theta_boat: Vector3
    dt_theta_boat: Vector3
    theta_rudder: float
    dt_theta_rudder: float
    theta_sail: float
    dt_theta_sail: float
    wind: Vector2


class SimResetInfo(TypedDict):
    min_position: Vector3
    max_position: Vector3


class LSASim(metaclass=ProfilingMeta):
    DEFAULT_PORT = 5555  # set in Dockerfile
    DOCKER_IMAGE_NAME = "vtautoboat/autoboat_simulation:latest"

    def __init__(self, name="default") -> None:
        self.name = re.sub(r"[^a-zA-Z0-9]", "-", name)

        self.wind = None
        self.sim_rate = None
        self.container = None
        self.port = None
        self.socket = None

        self.timer = None

        self.auto_pause_if_inactive = AutoPauseIfInactive(self.__pause_if_needed, self.__resume_if_needed)

        self.__init_simulation()

    def reset(self, wind: np.ndarray[2], water: np.ndarray[2], sim_rate: int):
        if is_debugging():
            print(f"[LSASim] Resetting simulation with wind {wind}, water {water} and sim_rate {sim_rate}")
        self.__send_msg(
            {
                "reset": {
                    "wind": {"x": wind[0], "y": wind[1]},
                    "water": {"x": water[0], "y": water[1]},
                    "freq": sim_rate,
                }
            }
        )
        msg = self.__recv_msg()
        obs = self.__parse_sim_obs(msg["obs"])
        info = self.__parse_sim_reset_info(msg["info"])
        return obs, info

    def step(self, wind: np.ndarray[2], water: np.ndarray[2], action: Action):
        if is_debugging():
            print(f"[LSASim] Sending action {action}")

        # print("sending message to sim")
        self.__send_msg(
            {
                "action": {
                    "theta_rudder": action["theta_rudder"].item(),
                    "theta_sail": action["theta_sail"].item(),
                    "wind": {"x": wind[0], "y": wind[1]},
                    "water": {"x": water[0], "y": water[1]},
                }
            }
        )
        msg = self.__recv_msg()
        obs = self.__parse_sim_obs(msg["obs"])
        done = msg["done"]
        return obs, done, msg["info"]

    def close(self):
        if is_debugging():
            print("[LSASim] Closing simulation")
        self.__send_msg({"close": True})
        self.__recv_msg()

    def stop(self):
        with DurationProgress(total=5, desc="Stopping docker container"):
            with self.auto_pause_if_inactive:
                self.container.kill()

    def __pause_if_needed(self):
        try:
            self.container.pause()
        except docker.errors.APIError as e:
            if is_debugging_all():
                raise e

    def __resume_if_needed(self):
        try:
            self.container.unpause()
        except docker.errors.APIError as e:
            if is_debugging_all():
                raise e

    def __init_simulation(self):
        if is_debugging():
            print(f"[LSASim] Launching docker container for {self.name}")
        self.__pull_image_if_needed()
        self.container, self.port = self.__launch_or_get_container(self.name)  # noqa
        self.__wait_until_ready()
        self.socket = self.__create_connection()
        self.__pause_if_needed()

    def __parse_sim_obs(self, obs: SimObservation) -> Observation:
        return {
            "p_boat": np.array([obs["p_boat"]["x"], obs["p_boat"]["y"], obs["p_boat"]["z"]], dtype=np.float32),
            "dt_p_boat": np.array([obs["dt_p_boat"]["x"], obs["dt_p_boat"]["y"], obs["dt_p_boat"]["z"]], dtype=np.float32),
            "theta_boat": np.array([obs["theta_boat"]["x"], obs["theta_boat"]["y"], obs["theta_boat"]["z"]], dtype=np.float32),
            "dt_theta_boat": np.array(
                [obs["dt_theta_boat"]["x"], obs["dt_theta_boat"]["y"], obs["dt_theta_boat"]["z"]], dtype=np.float32
            ),
            "theta_rudder": np.array([obs["theta_rudder"]], dtype=np.float32),
            "dt_theta_rudder": np.array([obs["dt_theta_rudder"]], dtype=np.float32),
            "theta_sail": np.array([obs["theta_sail"]], dtype=np.float32),
            "dt_theta_sail": np.array([obs["dt_theta_sail"]], dtype=np.float32),
            "wind": np.array([obs["wind"]["x"], obs["wind"]["y"]], dtype=np.float32),
            # Changed by autoboat: we do not care about water measurements and we have no way of measuring it, so it will mess with the RL algorithm
            # 'water': np.array([obs['water']['x'], obs['water']['y']], dtype=np.float32),
            "water": np.array([0, 0], dtype=np.float32),
        }

    def __parse_sim_reset_info(self, info: SimResetInfo) -> ResetInfo:
        min_pos = np.array([info["min_position"]["x"], info["min_position"]["y"], 0])
        max_pos = np.array([info["max_position"]["x"], info["max_position"]["y"], 1])
        map_bounds = np.array([min_pos, max_pos], dtype=np.float32)
        return {
            "map_bounds": map_bounds,
        }

    def __get_available_port(self):
        def get_random_port():
            # https://stackoverflow.com/a/46023565
            random_int = int.from_bytes(os.urandom(2), byteorder="big")
            return random_int % (65535 - 49152) + 49152

        port = get_random_port()
        while True:
            try:
                context = zmq.Context()
                socket = context.socket(zmq.REQ)
                socket.bind(f"tcp://*:{port}")
                socket.close()
                return port
            except zmq.error.ZMQError:
                port = get_random_port()

    def __pull_image_if_needed(self):
        client = docker.from_env()

        # Check if image is already pulled
        images = [img.tags for img in client.images.list()]
        if any(self.DOCKER_IMAGE_NAME in tags for tags in images):
            return

        for progress_dict in client.api.pull(self.DOCKER_IMAGE_NAME, stream=True, decode=True):
            status = progress_dict.get("status")
            progress = progress_dict.get("progress")

            if progress:
                sys.stdout.write(f"\r{status}: {progress}")
                sys.stdout.flush()



    def __launch_or_get_container(self, name):
        with DurationProgress(total=7, desc="Launching docker container"):
            try:
                client = docker.from_env()
            except docker.errors.DockerException as e:
                if is_debugging():
                    raise e
                raise RuntimeError(
                    "Docker socket is not detected. Please start docker and try again or make sure that you have correctly installed docker (MacOS: refer to this instruction https://stackoverflow.com/a/76125150)."
                ) from e

            name = f"sailboat-sim-lsa-gym-{name}"

            # try to find an existing container with the given name
            try:
                container = client.containers.get(name)
            except docker.errors.NotFound:
                container = None
            if container:
                port = container.attrs["NetworkSettings"]["Ports"][f"{self.DEFAULT_PORT}/tcp"][0]["HostPort"]
                if is_debugging():
                    print(f"\n[LSASim] Found existing docker container {name} running on port {port}")
                return container, port

            # find an available port
            port = self.__get_available_port()

            # launch a new container if none found or the existing container is not running
            try:
                print("launching new container")
                container = client.containers.run(
                    self.DOCKER_IMAGE_NAME,
                    name=name,
                    detach=True,
                    auto_remove=True,
                    # environment= {"DISPLAY": "unix:0"},
                    # device_requests=[
                    #     DeviceRequest(device_ids=["0"], capabilities=[['gpu']])
                    # ],
                    ports={
                        f"{self.DEFAULT_PORT}/tcp": port,
                        "22/tcp": None,
                    },
                )
            except docker.errors.NotFound as e:
                raise RuntimeError(
                    f"Could not find docker image {self.DOCKER_IMAGE_NAME}. Please make sure the image exists and try again."
                ) from e
            except docker.errors.APIError as e:
                raise RuntimeError(
                    f"Error communicating with Docker API: {str(e)}. "
                    "Please check that the Docker daemon is running and try again."
                ) from e
            except docker.errors.ContainerError as e:
                raise RuntimeError(
                    f"Container exited with non-zero exit code: {str(e)}. Please check the container logs for more information."
                ) from e
            except docker.errors.ImageNotFound as e:
                raise RuntimeError(
                    f"Image not found: {str(e)}. Please check that the image exists on the Docker registry and try again."
                ) from e

        return container, port

    def __wait_until_ready(self):
        with DurationProgress(total=17, desc="Waiting for docker container to be ready"):
            while True:
                logs = self.container.logs().decode("utf-8")
                if "INTENTIFIED CONTROL!" in logs:
                    break
                time.sleep(1)

    def __create_connection(self):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect(f"tcp://localhost:{self.port}")
        return socket

    def __send_msg(self, msg):
        with self.auto_pause_if_inactive:
            self.socket.send(msgpack.packb(msg))

    def __recv_msg(self):
        with self.auto_pause_if_inactive:
            msg = msgpack.unpackb(self.socket.recv(), raw=False)
            if "error" in msg:
                raise RuntimeError(msg["error"])
            return msg
