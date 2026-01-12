from typing import Any

from rclpy.impl.rcutils_logger import RcutilsLogger

from .base_autopilot import BaseAutopilot


class MotorboatAutopilot(BaseAutopilot):
    """
    A class containing algorithms to control a motorboat given sensor data.

    Inherits
    -------
    ``BaseAutopilot``
    """

    def __init__(self, parameters: dict[str, Any], logger: RcutilsLogger) -> None:
        """
        Parameters
        ----------
        parameters
            Dictionary that should contain the information from the ``config/motorboat_default_parameters.json`` file.

        logger
            A logger to use instead of print statements which works a little better with ROS.
            For more information see:
            https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html
        """

        super().__init__(parameters, logger)
