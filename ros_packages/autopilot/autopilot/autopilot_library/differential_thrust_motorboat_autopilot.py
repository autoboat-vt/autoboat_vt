from typing import Any

import numpy as np

from rclpy.impl.rcutils_logger import RcutilsLogger

from .utils.constants import PropellerMotorControlMode
from .utils.discrete_pid import DiscretePID
from .utils.position import Position
from .utils.utils_function_library import get_bearing, get_distance_between_angles, get_distance_between_positions

__all__ = ["DifferentialThrustMotorboatAutopilot"]


class DifferentialThrustMotorboatAutopilot:
    """Control a motorboat using independently commanded port and starboard thrusters."""

    def __init__(self, parameters: dict[str, Any], logger: RcutilsLogger) -> None:
        self.parameters = parameters
        self.logger = logger
        self.waypoints: list[Position] | None = None
        self.current_waypoint_index = 0
        self.heading_pid_controller = self._create_heading_pid()

    def _create_heading_pid(self) -> DiscretePID:
        return DiscretePID(
            sample_period=1 / self.parameters["autopilot_refresh_rate"],
            k_p=self.parameters["heading_p_gain"],
            k_i=self.parameters["heading_i_gain"],
            k_d=self.parameters["heading_d_gain"],
            n=self.parameters["heading_n_gain"],
        )

    def reset(self) -> None:
        """Reset waypoint tracking and heading-controller history."""
        self.heading_pid_controller = self._create_heading_pid()
        self.waypoints = None
        self.current_waypoint_index = 0

    def update_waypoints_list(self, waypoints_list: list[Position]) -> None:
        """Replace the current mission route."""
        self.waypoints = waypoints_list
        self.current_waypoint_index = 0

    def get_optimal_turn_force(self, heading: float, desired_heading: float) -> float:
        """
        Compute the PID heading correction used to steer with differential thrust.

        Parameters
        ----------
        heading
            The current heading of the boat measured counter-clockwise from true east.
        desired_heading
            The current desired heading of the boat measured counter-clockwise from true east.

        Returns
        -------
        `float`
            The heading correction in degrees.
        """

        self.heading_pid_controller.set_gains(
            k_p=self.parameters["heading_p_gain"],
            k_i=self.parameters["heading_i_gain"],
            k_d=self.parameters["heading_d_gain"],
            n=self.parameters["heading_n_gain"],
            sample_period=1 / self.parameters["autopilot_refresh_rate"],
        )

        error = get_distance_between_angles(desired_heading, heading)
        turn_force = self.heading_pid_controller(error)
        return float(
            np.clip(
                -turn_force,
                self.parameters["min_turn_force"],
                self.parameters["max_turn_force"],
            )
        )

    def _compute_differential_motor_commands(
        self, throttle: float, steering: float, control_type: str) -> tuple[float, float]:
        """Mix forward/reverse throttle and steering into port and starboard commands."""

        left = throttle + steering
        right = throttle - steering

        if control_type == "rpm":
            max_command = self.parameters["max_rpm"]
        else:
            max_command = 100.0
        return (
            float(np.clip(left, -max_command, max_command)),
            float(np.clip(right, -max_command, max_command)),
        )

    def _get_throttle_control(
        self, joystick_left_y: float, control_mode: PropellerMotorControlMode
    ) -> tuple[str, float]:
        if control_mode == PropellerMotorControlMode.RPM:
            return "rpm", self.parameters["max_rpm"] * joystick_left_y / 100.0
        if control_mode == PropellerMotorControlMode.DUTY_CYCLE:
            return "duty_cycle", joystick_left_y
        if control_mode == PropellerMotorControlMode.CURRENT:
            return "current", joystick_left_y
        return "rpm", 0.0

    def run_rc_control(
        self,
        joystick_left_y: float,
        joystick_right_x: float,
        propeller_motor_control_mode: PropellerMotorControlMode,
    ) -> tuple[str, float, float]:
        """Map RC throttle and steering inputs into two motor commands."""
        control_type, throttle = self._get_throttle_control(joystick_left_y, propeller_motor_control_mode)
        steering_scale = self.parameters["max_rpm"] if control_type == "rpm" else 100.0
        steering = -steering_scale * joystick_right_x / 100.0
        left, right = self._compute_differential_motor_commands(throttle, steering, control_type)
        return control_type, left, right

    def run_heading_hold_step(
        self,
        joystick_left_y: float,
        heading: float,
        desired_heading: float,
        propeller_motor_control_mode: PropellerMotorControlMode,
    ) -> tuple[str, float, float]:
        """Hold a target heading while taking throttle from the RC transmitter."""
        control_type, throttle = self._get_throttle_control(joystick_left_y, propeller_motor_control_mode)
        max_correction = max(
            abs(self.parameters["min_turn_force"]),
            abs(self.parameters["max_turn_force"]),
        )
        max_command = self.parameters["max_rpm"] if control_type == "rpm" else 100.0
        heading_correction = self.get_optimal_turn_force(heading, desired_heading)
        steering = max_command * heading_correction / max_correction if max_correction else 0.0
        left, right = self._compute_differential_motor_commands(throttle, steering, control_type)
        return control_type, left, right

    def run_waypoint_mission_step(self, current_position: Position, heading: float) -> tuple[float, float, float | None]:
        """Return port RPM, starboard RPM, and the steering correction for the active waypoint."""

        if not self.waypoints:
            return 0.0, 0.0, None

        target = self.waypoints[self.current_waypoint_index]
        distance = get_distance_between_positions(current_position, target)
        if distance <= self.parameters["waypoint_accuracy"]:
            if self.current_waypoint_index + 1 >= len(self.waypoints):
                self.reset()
                return 0.0, 0.0, None
            self.current_waypoint_index += 1
            return 0.0, 0.0, 0.0

        desired_heading = get_bearing(current_position, target)
        heading_correction = self.get_optimal_turn_force(heading, desired_heading)
        max_correction = max(
            abs(self.parameters["min_turn_force"]),
            abs(self.parameters["max_turn_force"]),
        )
        max_rpm = self.parameters["max_rpm"]

        if max_correction:
            steering_rpm = max_rpm * heading_correction / max_correction
        else:
            steering_rpm = 0.0

        min_rpm = self.parameters["min_rpm"]
        rpm_decay_rate = self.parameters["rpm_decay_rate"]

        rpm = + (max_rpm - min_rpm) * np.exp(-rpm_decay_rate * abs(heading_correction))
        left, right = self._compute_differential_motor_commands(float(rpm), steering_rpm, "rpm")

        return left, right, heading_correction
