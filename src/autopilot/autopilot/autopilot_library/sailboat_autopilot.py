from .discrete_pid import Discrete_PID
from .utils import *
from rclpy.impl.rcutils_logger import RcutilsLogger


class SailboatAutopilot:
    """
    Controls the boat based on a hard coded policy.
    This class contains all of the code to control a sailboat given an observation.

    The main method you should care about is the step method at the very bottom 
    which takes in all of the observations and outputs the correct sail and rudder angle to navigate to the next waypoint

    This class is used by the autopilot_node to control the boat through a ros topic
    """
    
    def __init__(self, parameters: dict, logger: RcutilsLogger):
        """
        parameters is a dictionary that contains information from the config/parameters.yaml file.
        For more information on specific parameters you are allow to use, please see that file
        """

        self.rudder_pid_controller = Discrete_PID(
            sample_period=(1 / parameters['autopilot_refresh_rate']), 
            Kp=parameters['heading_p_gain'], Ki=parameters['heading_i_gain'], Kd=parameters['heading_d_gain'], n=parameters['heading_n_gain'], 
        )
        
        self.parameters = parameters
        self.logger = logger
        self.waypoints: list[Position] = None
        self.current_waypoint_index = 0
        
        self.current_state = SailboatStates.NORMAL
        
        self.desired_tacking_angle = 0
   
   
    def reset(self): 
        self.__init__(parameters=self.parameters, logger=self.logger)
    
    def update_waypoints_list(self, waypoints_list: list[Position]):
        self.waypoints = waypoints_list
        self.current_waypoint_index = 0
        

    def get_optimal_sail_angle(self, apparent_wind_angle: float):
        """
        Runs a single step by using the sail lookup table. No side effects. Apparent wind angle is measured ccw from the centerline of the boat.
        
        Doesn't return an exit code because there is no reason why this should fail and this part of the code doesn't figure out if the boat has reached the waypoint
        Returns the desired sail angle and rudder angle as a tuple given the current observations
        """
    
        # 180 means wind pushing you backwards, 90 for the sail means let the sails all the way out
        # these are for close hauled, close reach, beam reach, broad reach and running respectively
        # the angles were estimated from a sailing position diagram and adam should probably take a look and move things around as he sees fit

        sail_positions = self.parameters['sail_lookup_table_sail_positions']
        wind_angles = self.parameters['sail_lookup_table_wind_angles']
        
        left = max(filter(lambda pos: pos <= float(apparent_wind_angle), wind_angles))
        right = min(filter(lambda pos: pos >= float(apparent_wind_angle), wind_angles))

        left = wind_angles.index(left)
        right = wind_angles.index(right)
        
        sail_angle = 0
        if (left == right):
            for i in range(len(sail_positions)):
                if float(apparent_wind_angle) == wind_angles[i]:
                    sail_angle = sail_positions[i]
        else:
            slope = (sail_positions[right] - sail_positions[left])/(wind_angles[right] - wind_angles[left])
            sail_angle = slope * (float(apparent_wind_angle) - wind_angles[left]) + sail_positions[left]
        
        return sail_angle
        
        
    def get_optimal_rudder_angle(self, heading, desired_heading):
        error = get_distance_between_angles(desired_heading, heading)
        
        self.rudder_pid_controller.set_gains(
            Kp=self.parameters['heading_p_gain'], Ki=self.parameters['heading_i_gain'], Kd=self.parameters['heading_d_gain'], 
            n=self.parameters['heading_n_gain'], sample_period=self.parameters['autopilot_refresh_rate']
        )
        
        rudder_angle = self.rudder_pid_controller(error)
        rudder_angle = np.clip(rudder_angle, self.parameters['min_rudder_angle'], self.parameters['max_rudder_angle'])
        return rudder_angle
    
    
    
    
    
    def get_decision_zone_size(self, distance_to_waypoint):
        tack_distance = self.parameters['tack_distance']
        no_sail_zone_size = self.parameters['no_sail_zone_size']
        
        inner = (tack_distance/distance_to_waypoint) * np.sin(np.deg2rad(no_sail_zone_size/2))
        inner = np.clip(inner, -1, 1)
        return np.clip(np.rad2deg(np.arcsin(inner)), 0, no_sail_zone_size)
        
        
    def apply_decision_zone_tacking_logic(self, heading, desired_heading, true_wind_angle, apparent_wind_angle, distance_to_waypoint):
        """
        If you don't know how this works (you don't) feel free to ask Chris about this and he can explain it to you.
        I am sorry to anyone who has to try to understand how this algorithm works. I was in your shoes once...
        
        Returns:
        - The angle that the boat should be holding CCW from true east (which you can think of as a modified desired heading that won't go upwind)
        - Whether the boat would need to tack to reach that heading
        """
        
        global_true_wind_angle = (heading + true_wind_angle) % 360
        global_true_up_wind_angle = (global_true_wind_angle + 180) % 360   # goes in the opposite direction of the global true wind angle
        
        global_apparent_wind_angle = (heading + apparent_wind_angle) % 360
        global_apparent_up_wind_angle = (global_apparent_wind_angle + 180) % 360   # goes in the opposite direction of the global apparent wind angle


        no_sail_zone_bounds = (
            (global_apparent_up_wind_angle - self.parameters['no_sail_zone_size']/2) % 360, # lower
            (global_apparent_up_wind_angle + self.parameters['no_sail_zone_size']/2) % 360  # upper
        )

        decision_zone_size = self.get_decision_zone_size(distance_to_waypoint)
        decision_zone_bounds = (
            (global_true_up_wind_angle - decision_zone_size/2) % 360, # lower
            (global_true_up_wind_angle + decision_zone_size/2) % 360  # upper
        )
    
    
        # If desired heading it is not in any of the zones
        if not is_angle_between_boundaries(desired_heading, no_sail_zone_bounds[0], no_sail_zone_bounds[1]): 
            if get_maneuver_from_desired_heading(heading, desired_heading, true_wind_angle) == SailboatManeuvers.TACK:
                return desired_heading, True
            else:
                return desired_heading, False


        # If desired heading is in zone 1
        if is_angle_between_boundaries(desired_heading, decision_zone_bounds[1], no_sail_zone_bounds[1]):
            if (heading - global_true_up_wind_angle) % 360 < 180:   # Starboard side of true wind
                return no_sail_zone_bounds[1], False
                
            else:   # Port side of the true wind
                # sbtd tack
                return no_sail_zone_bounds[1], True
                   
                                
        # If desired heading is in zone 3
        if is_angle_between_boundaries(desired_heading, decision_zone_bounds[0], no_sail_zone_bounds[0]):
            if (heading - global_true_up_wind_angle) % 360 < 180:   # Starboard side of true wind
                # port tack
                return no_sail_zone_bounds[0], True
                
            else:   # Port side of the true wind
                return no_sail_zone_bounds[0], False
    
        
        # If desired heading in zone 2      
        distance_to_lower_no_sail_zone = abs(get_distance_between_angles(no_sail_zone_bounds[0], heading))
        distance_to_upper_no_sail_zone = abs(get_distance_between_angles(no_sail_zone_bounds[1], heading))
       
        if distance_to_lower_no_sail_zone < distance_to_upper_no_sail_zone: 
            return no_sail_zone_bounds[0], False
        
        else: 
            return no_sail_zone_bounds[1], False

        
        
    def run_waypoint_mission_step(self, cur_position: Position, global_velocity_vector: np.ndarray, heading: float, apparent_wind_vector: np.ndarray):
        """
        Assumes that there are waypoints inputted in the autopilot.
        
        takes in the following:
        - position as a Position object (from position.py)
        - global velocity as a numpy array with 2 elements (x, y) in meters/ second
        - heading as a float in degrees measured ccw from true east
        - apparent wind vector as a numpy array with 2 elements (x, y) in meters/ second. This captures data about AWA and AWS
        
        returns the sail angle, rudder angle, and exit code
        """
        
        if not self.waypoints: raise Exception("Expected route to be inputted into the autopilot. Field self.waypoints was not filled")

        boat_speed = np.sqrt(global_velocity_vector[0]**2 + global_velocity_vector[1]**2)

        boat_speed, global_velocity_angle = cartesian_vector_to_polar(global_velocity_vector[0], global_velocity_vector[1])
        local_velocity_angle = global_velocity_angle - heading
        local_velocity_vector = boat_speed * np.array([np.cos(np.deg2rad(local_velocity_angle)), np.sin(np.deg2rad(local_velocity_angle))])
        
        # https://en.wikipedia.org/wiki/Apparent_wind#/media/File:DiagramApparentWind.png
        true_wind_vector = apparent_wind_vector + local_velocity_vector
        
        true_wind_speed, true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])
        apparent_wind_speed, apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1])
        
        global_true_wind_angle = (true_wind_angle + heading) % 360
        
        self.logger.info(f"autopilot: {global_true_wind_angle}")
        
        desired_pos = self.waypoints[self.current_waypoint_index]
        distance_to_desired_position = get_distance_between_positions(cur_position, desired_pos)

        
        
        # Has Reached The Waypoint
        if distance_to_desired_position < self.parameters['waypoint_accuracy']: 
            
            if len(self.waypoints) <= self.current_waypoint_index + 1:    # Has Reached The Final Waypoint
                self.reset()
                return None, None
            
            self.current_waypoint_index += 1
            
        
        
        
        if self.current_state == SailboatStates.NORMAL:
            desired_heading = get_bearing(current_pos=cur_position, destination_pos=desired_pos)

            desired_heading, should_tack_condition1 = self.apply_decision_zone_tacking_logic(heading, desired_heading, true_wind_angle, apparent_wind_angle, distance_to_desired_position)
            
            global_true_up_wind_angle = (180 + global_true_wind_angle) % 360
            should_tack_condition2 = is_angle_between_boundaries(global_true_up_wind_angle, heading, desired_heading)
            
            if should_tack_condition1 or should_tack_condition2:
                optimal_rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
                
                self.desired_tacking_angle = desired_heading
                
                if optimal_rudder_angle > 0: 
                    self.current_state = SailboatStates.CW_TACKING  
                else: 
                    self.current_state = SailboatStates.CCW_TACKING

            
            rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
            sail_angle = self.get_optimal_sail_angle(apparent_wind_angle)
            
            
            

        elif self.current_state == SailboatStates.CW_TACKING or self.current_state == SailboatStates.CCW_TACKING:
            sail_angle = self.get_optimal_sail_angle(apparent_wind_angle)

            if self.current_state == SailboatStates.CW_TACKING:
                tack_direction = 1
            elif self.current_state == SailboatStates.CCW_TACKING:
                tack_direction = -1


            rudder_angle = self.parameters['rudder_hard_over'] * tack_direction
            
            if self.parameters['perform_forced_jibe_instead_of_tack']: 
                rudder_angle *= -1


            if abs((heading - self.desired_tacking_angle)) % 360 < self.parameters['tack_tolerance']: # if we have finished the tack
                self.current_state = SailboatStates.NORMAL
            
        
        else: 
            Exception("Unsupported State Transitioned Into")
        
        
        return sail_angle, rudder_angle
    
    
    
    
    
    def run_rc_control(self, joystick_left_y, joystick_right_x):
        # https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio 
        
        min_sail_angle, max_sail_angle = self.parameters['min_sail_angle'], self.parameters['max_sail_angle']
        sail_angle = (((joystick_left_y - -100) * (max_sail_angle - min_sail_angle)) / (100 - -100)) + min_sail_angle
        
        min_rudder_angle, max_rudder_angle = self.parameters['min_rudder_angle'], self.parameters['max_rudder_angle']
        rudder_angle = (((joystick_right_x - -100) * (max_rudder_angle - min_rudder_angle)) / (100 - -100)) + min_rudder_angle
    
        return sail_angle, rudder_angle
    