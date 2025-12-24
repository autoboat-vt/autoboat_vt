from typing import Any

from rclpy.impl.rcutils_logger import RcutilsLogger

from .discrete_pid import Discrete_PID
from .utils import *


class MotorboatAutopilot:
    """
    This class contains the main control algorithms to control a motorboat given sensor data.

    As the person using this class, you generally should not have to worry about any of the functions
    that start with an underscore such as `_function_name`. These functions are meant to be private to
    the class and are essentially helper functions.

    This class is mainly used by the Motorboat Autopilot Node to control the boat through a ROS topic.
    """

    def __init__(self, parameters: dict[str, Any], logger: RcutilsLogger) -> None:
        """
        Parameters
        ----------
        parameters
            Dictionary that should contain the information from the `config/motorboat_default_parameters.yaml` file.

        logger
            A logger to use instead of print statements which works a little better with ROS.
            For more information see https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html.
        """

        self.rudder_pid_controller = Discrete_PID(
            sample_period=(1 / parameters["autopilot_refresh_rate"]),
            Kp=parameters["heading_p_gain"],
            Ki=parameters["heading_i_gain"],
            Kd=parameters["heading_d_gain"],
            n=parameters["heading_n_gain"],
        )

        self.autopilot_parameters = parameters
        self.logger = logger
        self.waypoints: list[Position] = []
        self.current_state: SailboatStates = SailboatStates.NORMAL

        self.desired_tacking_angle: float = 0.0
        self.current_waypoint_index: int = 0

    def reset(self) -> None:
        """
        Reinitializes the MotorboatAutopilot with the same parameters and the same logger.
        This essentially resets things like waypoints.
        """

        self.__init__(self.autopilot_parameters, self.logger)

    def update_waypoints_list(self, waypoints_list: list[Position]) -> None:
        """
        Updates the list of waypoints that the motorboat should follow.

        Parameters
        ----------
        waypoints_list
            A list of Position objects that represent the waypoints the motorboat should follow in order.
        """
