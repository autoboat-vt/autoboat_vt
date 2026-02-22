
from .discrete_pid import Discrete_PID
from .utils import *
from rclpy.impl.rcutils_logger import RcutilsLogger
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

        self.rpm_pid_controller = Discrete_PID(
            sample_period=(1 / autopilot_parameters['autopilot_refresh_rate']), 
            Kp=autopilot_parameters['rpm_p_gain'], Ki=autopilot_parameters['rpm_i_gain'], 
            Kd=autopilot_parameters['rpm_d_gain'], n=autopilot_parameters['rpm_n_gain'], 
        )
        
        self.autopilot_parameters = autopilot_parameters
        self.logger = logger
        self.waypoints: list[Position] = None        
        self.current_state = SailboatStates.NORMAL
        
        self.desired_tacking_angle = 0
        self.current_waypoint_index = 0


        self.propeller_rpm = 0.0
        self.rudder_angle = 0.0
        
    
    def reset(self):
        """
        Reinitializes the MotorboatAutopilot with the same parameters and the same logger. This essentially resets things like waypoints. 
        """ 
        self.__init__(self.autopilot_parameters, logger=self.logger)

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
        self.waypoints = waypoints_list
        self.current_waypoint_index = 0



        
    def _get_decision_zone_size(self, distance_to_waypoint: float) -> float:
        """
        Check out this for more information on decision zones: https://autoboat-vt.github.io/autoboat_docs/ros2_packages/autopilot_package/sailboat_autopilot/.
        I would not be able to explain it here.
        Args:
            distance_to_waypoint (float): the distance to the next waypoint in meters

        Returns:
            float: the total size of the decision zone in degrees
        """
        tack_distance = self.autopilot_parameters['tack_distance']
        no_sail_zone_size = self.autopilot_parameters['no_sail_zone_size']
        
        inner = (tack_distance/distance_to_waypoint) * np.sin(np.deg2rad(no_sail_zone_size/2))
        inner = np.clip(inner, -1, 1)
        return np.clip(np.rad2deg(np.arcsin(inner)), 0, no_sail_zone_size)
            
    

    def get_optimal_rudder_angle(self, heading: float, desired_heading: float) -> float:
        """
        Args:
            heading (float): the current heading of the boat measured counter-clockwise from true east
            desired_heading (float): the current desired heading of the boat measured counter-clockwise from true east

        Returns:
            float: the angle we should turn the rudder in order to turn from our current heading to the desired heading
        """
        # Update the gains of the controller in case they changed. If the gains didn't change, then nothing happens
        self.rudder_pid_controller.set_gains(
            Kp=self.autopilot_parameters['heading_p_gain'], Ki=self.autopilot_parameters['heading_i_gain'], Kd=self.autopilot_parameters['heading_d_gain'], 
            n=self.autopilot_parameters['heading_n_gain'], sample_period=self.autopilot_parameters['autopilot_refresh_rate']
        )
        
        error = get_distance_between_angles(desired_heading, heading)
        rudder_angle = self.rudder_pid_controller(error)
        rudder_angle = np.clip(rudder_angle, self.autopilot_parameters['min_rudder_angle'], self.autopilot_parameters['max_rudder_angle'])
        return rudder_angle

    def get_optimal_rpm(self, current_position: float ,desired_position: float) ->float:
        
        self.rpm_pid_controller.set_gains(
            Kp=self.autopilot_parameters['rpm_p_gain'], Ki=self.autopilot_parameters['rpm_i_gain'], Kd=self.autopilot_parameters['rpm_d_gain'], 
            n=self.autopilot_parameters['rpm_n_gain'], sample_period=self.autopilot_parameters['autopilot_refresh_rate']
        )
        error = get_distance_between_positions(desired_position,current_position)
        rpm = self.rpm_pid_controller(error)
        rpm = np.clip(rpm, 0.0,300.0)

        return rpm

    

    def run_waypoint_mission_step(
         self, 
            current_position: Position, 
            # global_velocity_vector: np.ndarray, 
            heading: float, 
        ) -> tuple[float, float]:
        """
        Assumes that there are waypoints inputted in the autopilot.
        
        Args:
            current_position (Position): a Position object from position.py that represents the boat's current latitude and longitude position
            # global_velocity_vector (np.ndarray): global velocity as a numpy array with 2 elements (x, y) in meters/ second
            heading (float): direction the boat is facing in degrees measured counter-clockwise from true east

        Returns:
            tuple[float, float]: (propeller_rpm, rudder_angle) that the autopilot believes that we should take
        """

    
        if not self.waypoints: raise Exception("Expected route to be inputted into the autopilot. Field self.waypoints was not filled")

        desired_position = self.waypoints[self.current_waypoint_index]
        distance_to_desired_position = get_distance_between_positions(current_position, desired_position)

        self.logger.info(f"Current Position : {current_position.get_longitude_latitude()}")
        self.logger.info(f"Desired Position : {desired_position.get_longitude_latitude()}")

        desired_heading = get_bearing(current_position, desired_position)
        self.rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
        self.propeller_rpm = self.get_optimal_rpm(current_position,desired_position)

        self.logger.info(f"Bearing: {desired_heading}")
        if distance_to_desired_position < self.autopilot_parameters['waypoint_accuracy']: 
            self.rudder_angle = 0.0
            self.propeller_rpm = 0.0
            if len(self.waypoints) <= self.current_waypoint_index + 1:    # Has Reached The Final Waypoint
                self.reset()
                return None, None
            self.current_waypoint_index += 1
            desired_position = self.waypoints[self.current_waypoint_index]

            
        return self.propeller_rpm, self.rudder_angle
    
    # def find_closest_waypoint(self, current_position):
    #     closest_waypoint = self.waypoints[0]
    #     closest_distance = get_distance_between_positions(closest_waypoint, current_position)
    #     for waypoint  in self.waypoints:
    #         if(get_distance_between_positions(waypoint, current_position) <= closest_distance):
    #             closest_waypoint = waypoint


    #     return way


        
