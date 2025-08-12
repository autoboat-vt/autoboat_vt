from .discrete_pid import Discrete_PID
from .utils import *
from rclpy.impl.rcutils_logger import RcutilsLogger


class MotorboatAutopilot:
    
    def __init__(self, parameters, logger):
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
            sample_period=(1 / parameters['autopilot_refresh_rate']), 
            Kp=parameters['heading_p_gain'], Ki=parameters['heading_i_gain'], Kd=parameters['heading_d_gain'], n=parameters['heading_n_gain'], 
        )
        
        self.parameters = parameters
        self.logger = logger
        self.waypoints: list[Position] = None        
        self.current_state = SailboatStates.NORMAL
        
        self.desired_tacking_angle = 0
        self.current_waypoint_index = 0
        
    
    def reset(self):
        """
        Reinitializes the MotorboatAutopilot with the same parameters and the same logger. This essentially resets things like waypoints. 
        """ 

        pass
    
    def update_waypoints_list(self, waypoints_list: list[Position]) -> None:
        """
        Args:
            waypoints_list (list[Position]): a list of all the waypoints you would like the boat to follow
        """
        pass