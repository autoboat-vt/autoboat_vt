
from typing import Any

import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from scipy.interpolate import interp1d

from .utils.discrete_pid import DiscretePID
from .utils.position import Position
from .utils.utils_function_library import get_bearing, get_distance_between_angles, get_distance_between_positions


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

        self.heading_pid_controller = DiscretePID(
            sample_period=(1 / parameters["autopilot_refresh_rate"]),
            k_p=parameters["heading_p_gain"],
            k_i=parameters["heading_i_gain"],
            k_d=parameters["heading_d_gain"],
            n=parameters["heading_n_gain"],
        )
        
        self.parameters = parameters
        self.logger = logger
        self.waypoints: list[Position] = None
        
        self.current_waypoint_index = 0

        

    def reset(self) -> None:
        """Resets the autopilot to its initial state."""

        self.heading_pid_controller = DiscretePID(
            sample_period=(1 / self.parameters["autopilot_refresh_rate"]),
            k_p=self.parameters["heading_p_gain"],
            k_i=self.parameters["heading_i_gain"],
            k_d=self.parameters["heading_d_gain"],
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
        self.waypoints = waypoints_list
        self.current_waypoint_index = 0
    
    

    def get_optimal_rudder_angle(self, heading: float, desired_heading: float) -> float:
        """
        Gets the optimal rudder angle that will help the boat go from its current heading to the desired_heading
        using a PID controller.
        
        Parameters
        ----------
            heading (float): the current heading of the boat measured counter-clockwise from true east.
            desired_heading (float): the current desired heading of the boat measured counter-clockwise from true east.

        Returns
        -------
            float: the angle we should turn the rudder in order to turn from our current heading to the desired heading
        """
        # Update the gains of the controller in case they changed. If the gains didn't change, then nothing happens
        self.heading_pid_controller.set_gains(
            k_p=self.parameters['heading_p_gain'], k_i=self.parameters['heading_i_gain'], k_d=self.parameters['heading_d_gain'],
            n=self.parameters['heading_n_gain'], sample_period=self.parameters['autopilot_refresh_rate']
        )
        
        error = get_distance_between_angles(desired_heading, heading)
        rudder_angle = self.heading_pid_controller(error)
        return np.clip(rudder_angle, self.parameters['min_rudder_angle'], self.parameters['max_rudder_angle'])



    def get_optimal_rpm(self, rudder_angle: float) ->float:
        """
        Gets the optimal motor RPM that will help it arrive at the destination waypoint efficiently.
        
        Parameters
        ----------
            rudder_angle (float): the angle the rudder is currently at

        Returns
        -------
            float: The rpm the boat should use to get to the desired heading
        """
        

        error = abs(rudder_angle)
        
        # if the error is high, then we want to be going at min rpm and if the error is low,
        # then we want to be going at max rpm
        max_rpm = self.parameters['max_rpm']
        min_rpm = self.parameters['min_rpm']
        
        rpm_output = min_rpm + (max_rpm - min_rpm) * np.exp(-self.parameters["rpm_decay_rate"] * error)
        
        # self.logger.info(f"Error: {error}, RPM Output: {rpm_output}")
        
        return float(np.clip(rpm_output, min_rpm, max_rpm))
    

    def run_waypoint_mission_step(self, current_position: Position, heading: float) -> tuple[float, float]:
        """
        Assumes that there are waypoints loaded in the autopilot.
        
        Parameters
        ----------
            current_position (Position): a Position object from position.py thatrepresents the boat's current
                latitude and longitude position
            heading (float): direction the boat is facing in degrees measured counter-clockwise from true east

        Returns
        -------
            tuple[float, float]: (propeller_rpm, rudder_angle) that the autopilot believes that we should take
        """

    
        if not self.waypoints:
            raise Exception("Expected route to be loaded into the autopilot. Field self.waypoints was not filled")

        desired_position = self.waypoints[self.current_waypoint_index]
        distance_to_desired_position = get_distance_between_positions(current_position, desired_position)

        desired_heading = get_bearing(current_position, desired_position)

        rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
        propeller_rpm = self.get_optimal_rpm(rudder_angle)


        if distance_to_desired_position < self.parameters['waypoint_accuracy']:
            rudder_angle = 0.0
            propeller_rpm = 0.0
            
            if len(self.waypoints) <= self.current_waypoint_index + 1:    # Has Reached The Final Waypoint
                self.reset()
                return 0.0, 0.0
            
            self.current_waypoint_index += 1
            desired_position = self.waypoints[self.current_waypoint_index]

            
        return propeller_rpm, rudder_angle
