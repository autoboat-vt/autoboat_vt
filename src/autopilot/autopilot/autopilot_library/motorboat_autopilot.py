from typing import Any

from rclpy.impl.rcutils_logger import RcutilsLogger

from .utils.discrete_pid import DiscretePID
from .utils.position import Position


class MotorboatAutopilot:
    """A class containing algorithms to control a motorboat given sensor data."""

    def __init__(self, parameters: dict[str, Any], logger: RcutilsLogger) -> None:
        """
        Parameters
        ----------
        parameters
            Dictionary that should contain information from a file in the ```config``` folder.

        logger
            A logger to use instead of print statements which works a little better with ROS.
            For more information see:
            https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html
        """

        self.rudder_pid_controller = DiscretePID(
            sample_period=(1 / parameters["autopilot_refresh_rate"]),
            Kp=parameters["heading_p_gain"],
            Ki=parameters["heading_i_gain"],
            Kd=parameters["heading_d_gain"],
            n=parameters["heading_n_gain"],
        )

        self.parameters = parameters
        self.logger = logger

        self.waypoints: list[Position] | None = None
        self.current_waypoint_index: int = 0


    def reset(self) -> None:
        """Resets the autopilot to its initial state."""

        self.rudder_pid_controller = DiscretePID(
            sample_period=(1 / self.parameters["autopilot_refresh_rate"]),
            Kp=self.parameters["heading_p_gain"],
            Ki=self.parameters["heading_i_gain"],
            Kd=self.parameters["heading_d_gain"],
            n=self.parameters["heading_n_gain"],
        )
        self.waypoints = None
        self.current_waypoint_index = 0


    def update_waypoints_list(self, waypoints_list: list[Position]) -> None:
        """
        Updates the list of waypoints that the boat should follow.

        Parameters
        ----------
        waypoints_list
            A list of ``Position`` objects that form the path the boat should follow.
        """
