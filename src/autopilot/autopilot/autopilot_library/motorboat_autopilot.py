
from .discrete_pid import Discrete_PID
from .utils import *
from rclpy.impl.rcutils_logger import RcutilsLogger
from typing import Any

class MotorboatAutopilot:
    """
    This class contains the main control algorithms to control a motorboat given sensor data.
    As the person using this class, you generally should not have to worry about any of the functions that start with an underscore such as _function_name.
    These functions are meant to be private to the class and are essentially helper functions. 
    
    This class is mainly used by the Motorboat Autopilot Node to control the boat through a ROS topic
    """
    
    def __init__(self, autopilot_parameters: dict[str, Any], logger: RcutilsLogger):
        """
        Args:
            parameters (dict): a dictionary that should contain the information from the config/motorboat_default_parameters.yaml file.
                For more information on specific parameters you are allowed to use, please see that file.
            
            logger (RcutilsLogger): A ROS logger to use instead of print statements which works a little better with ROS.
                For more information: https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html.
                This logger is what you get by running self.get_logger(). So for example in order to log an info message,
                please use logger.info("message").
        """


        self.rudder_pid_controller = Discrete_PID(
            sample_period=(1 / autopilot_parameters['autopilot_refresh_rate']), 
            Kp=autopilot_parameters['heading_p_gain'], Ki=autopilot_parameters['heading_i_gain'], 
            Kd=autopilot_parameters['heading_d_gain'], n=autopilot_parameters['heading_n_gain'], 
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

        pass
    
    def update_waypoints_list(self, waypoints_list: list[Position]) -> None:
        """
        Args:
            waypoints_list (list[Position]): a list of all the waypoints you would like the boat to follow
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
        rpm = np.clip(rpm, 0.0,100000.0)

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
            
            if len(self.waypoints) <= self.current_waypoint_index + 1:    # Has Reached The Final Waypoint
                self.reset()
                self.propeller_rpm = 0.0
                return None, None
            
            self.current_waypoint_index += 1
        return self.propeller_rpm, self.rudder_angle

        
