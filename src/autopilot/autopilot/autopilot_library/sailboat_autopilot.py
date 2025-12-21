from .discrete_pid import Discrete_PID
from .utils import *
from rclpy.impl.rcutils_logger import RcutilsLogger

from typing import Any


class SailboatAutopilot:
    """
    This class contains the main control algorithms for sailing using the decision zone tacking algorithm to control a sailboat given sensor data.
    As the person using this class, you generally should not have to worry about any of the functions that start with an underscore such as _function_name.
    These functions are meant to be private to the class and are essentially helper functions.

    This class is mainly used by the Sailboat Autopilot Node to control the boat through a ROS topic
    """

    def __init__(self, parameters: dict[str, Any], logger: RcutilsLogger):
        """
        Args:
            parameters (dict): a dictionary that should contain the information from the config/sailboat_default_parameters.yaml file.
                For more information on specific parameters you are allowed to use, please see that file.

            logger (RcutilsLogger): A ROS logger to use instead of print statements which works a little better with ROS.
                For more information: https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html.
                This logger is what you get by running self.get_logger(). So for example in order to log an info message,
                please run logger.info("message").
        """

        self.rudder_pid_controller = Discrete_PID(
            sample_period=(1 / parameters["autopilot_refresh_rate"]),
            Kp=parameters["heading_p_gain"],
            Ki=parameters["heading_i_gain"],
            Kd=parameters["heading_d_gain"],
            n=parameters["heading_n_gain"],
        )

        self.parameters = parameters
        self.logger = logger
        self.waypoints: list[Position] = None
        self.current_state = SailboatStates.NORMAL

        self.desired_tacking_angle = 0
        self.current_waypoint_index = 0

    def reset(self) -> None:
        """
        Reinitializes the SailboatAutopilot with the same parameters and the same logger. This essentially resets things like waypoints.
        """
        self.__init__(parameters=self.parameters, logger=self.logger)

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
        tack_distance = self.parameters["tack_distance"]
        no_sail_zone_size = self.parameters["no_sail_zone_size"]

        inner = (tack_distance / distance_to_waypoint) * np.sin(np.deg2rad(no_sail_zone_size / 2))
        inner = np.clip(inner, -1, 1)
        return np.clip(np.rad2deg(np.arcsin(inner)), 0, no_sail_zone_size)

    def _get_maneuver_from_desired_heading(self, heading: float, desired_heading: float, true_wind_angle: float) -> SailboatManeuvers:
        """
        A maneuver is basically a "mode" of sailing. During each of these "modes" we have to act differently, and this function
        helps us determine if we should switch sailing "modes". The three main types of sailing "modes" are STANDARD, TACK, and JIBE.
        In STANDARD, we are just trying to face our boat towards a waypoint and sail directly towards that point. When we are in the TACK
        "mode", our goal is to complete a TACK across the wind, and when we are in the JIBE "mode", our goal is to complete a JIBE across the
        wind. For more information about what tacking and jibing are, please read the following: https://captainsword.com/tacking-and-jibing/

        Args:
            heading (float): the direction the boat is currently facing measured in degrees counter-clockwise from true east
            desired_heading (float): the direction the boat wants to face measured in degrees counter-clockwise from true east
            true_wind_angle (float): the true wind angle in degrees measured counter-clockwise from the centerline of the boat

        Returns:
            SailboatManeuvers: Which maneuver the boat should take.
        """
        # Wind angle counter-clockwise from true east
        global_true_wind_angle = (true_wind_angle + heading) % 360

        # Calculate upwind angle (nominal angle at which course is heading straight into the wind)
        # Global True Wind Angle                    0     45    90   135  179 | 180  225  270  315  360
        # Global True Upwind Angle                  180   225   270  315  359 | 0    45   90   135  180
        global_true_upwind_angle = (global_true_wind_angle + 180) % 360

        if is_angle_between_boundaries(global_true_wind_angle, heading, desired_heading):
            return SailboatManeuvers.JIBE

        elif is_angle_between_boundaries(global_true_upwind_angle, heading, desired_heading):
            return SailboatManeuvers.TACK

        else:
            return SailboatManeuvers.STANDARD

    def _apply_decision_zone_tacking_logic(
        self,
        current_heading: float,
        desired_heading: float,
        true_wind_angle: float,
        apparent_wind_angle: float,
        distance_to_waypoint: float,
    ) -> tuple[float, bool]:
        """
        TODO MAKE DOCUMENTATION A LITTLE BETTER
        If you don't know how this works (you don't) feel free to ask Chris about this and he can explain it to you.
        I am sorry to anyone who has to try to understand how this algorithm works. I was in your shoes once...

        Args:
            heading (float): the direction the boat is currently facing measured in degrees counter-clockwise from true east
            desired_heading (float): the direction the boat wants to face measured in degrees counter-clockwise from true east
            true_wind_angle (float): the true wind angle in degrees measured counter-clockwise from the centerline of the boat
            apparent_wind_angle (float): the apparent wind angle in degrees measured counter-clockwise from the centerline of the boat
            distance_to_waypoint (float): the distance to the next waypoint in meters

        Returns:
            tuple[float, bool]: (The angle that the boat should be holding measured in degrees counter-clockwise from true east, Whether the boat would need to tack to reach that heading)
        """

        global_true_wind_angle = (current_heading + true_wind_angle) % 360
        global_true_up_wind_angle = (
            global_true_wind_angle + 180
        ) % 360  # goes in the opposite direction of the global true wind angle

        global_apparent_wind_angle = (current_heading + apparent_wind_angle) % 360
        global_apparent_up_wind_angle = (
            global_apparent_wind_angle + 180
        ) % 360  # goes in the opposite direction of the global apparent wind angle

        no_sail_zone_bounds = (
            (global_apparent_up_wind_angle - self.parameters["no_sail_zone_size"] / 2) % 360,  # lower bound
            (global_apparent_up_wind_angle + self.parameters["no_sail_zone_size"] / 2) % 360,  # upper bound
        )

        decision_zone_size = self._get_decision_zone_size(distance_to_waypoint)
        decision_zone_bounds = (
            (global_true_up_wind_angle - decision_zone_size / 2) % 360,  # lower bound
            (global_true_up_wind_angle + decision_zone_size / 2) % 360,  # upper bound
        )

        # If desired heading it is not in any of the zones
        if not is_angle_between_boundaries(desired_heading, no_sail_zone_bounds[0], no_sail_zone_bounds[1]):
            if self._get_maneuver_from_desired_heading(current_heading, desired_heading, true_wind_angle) == SailboatManeuvers.TACK:
                return desired_heading, True  # tack over to desired heading
            else:
                return desired_heading, False  # No tack

        # If desired heading is in zone 1
        if is_angle_between_boundaries(desired_heading, decision_zone_bounds[1], no_sail_zone_bounds[1]):
            # Starboard side of true wind
            if (current_heading - global_true_up_wind_angle) % 360 < 180:
                return no_sail_zone_bounds[1], False  # No tack

            # Port side of the true wind
            else:
                return no_sail_zone_bounds[1], True  # Starboard tack

        # If desired heading is in zone 3
        if is_angle_between_boundaries(desired_heading, decision_zone_bounds[0], no_sail_zone_bounds[0]):
            # Starboard side of true wind
            if (current_heading - global_true_up_wind_angle) % 360 < 180:
                return no_sail_zone_bounds[0], True  # Port tack

            # Port side of the true wind
            else:
                return no_sail_zone_bounds[0], False  # No tack

        # If desired heading in zone 2
        distance_to_lower_no_sail_zone = abs(get_distance_between_angles(no_sail_zone_bounds[0], current_heading))
        distance_to_upper_no_sail_zone = abs(get_distance_between_angles(no_sail_zone_bounds[1], current_heading))

        if distance_to_lower_no_sail_zone < distance_to_upper_no_sail_zone:
            return no_sail_zone_bounds[0], False  # No tack

        else:
            return no_sail_zone_bounds[1], False  # No tack

    def run_waypoint_mission_step(
        self, current_position: Position, global_velocity_vector: np.ndarray, heading: float, apparent_wind_vector: np.ndarray
    ) -> tuple[float, float]:
        """
        Assumes that there are waypoints inputted in the autopilot.

        Args:
            current_position (Position): a Position object from position.py that represents the boat's current latitude and longitude position
            global_velocity_vector (np.ndarray): global velocity as a numpy array with 2 elements (x, y) in meters/ second
            heading (float): direction the boat is facing in degrees measured counter-clockwise from true east
            apparent_wind_vector (np.ndarray): A numpy array with 2 elements (x, y) in meters/ second. Wind angle measured counter-clockwise from the centerline of the boat

        Returns:
            tuple[float, float]: (sail_angle, rudder_angle) that the autopilot believes that we should take
        """

        if not self.waypoints:
            raise Exception("Expected route to be inputted into the autopilot. Field self.waypoints was not filled")

        boat_speed = np.sqrt(global_velocity_vector[0] ** 2 + global_velocity_vector[1] ** 2)

        boat_speed, global_velocity_angle = cartesian_vector_to_polar(global_velocity_vector[0], global_velocity_vector[1])
        local_velocity_angle = global_velocity_angle - heading
        local_velocity_vector = boat_speed * np.array(
            [np.cos(np.deg2rad(local_velocity_angle)), np.sin(np.deg2rad(local_velocity_angle))]
        )

        # https://en.wikipedia.org/wiki/Apparent_wind#/media/File:DiagramApparentWind.png
        true_wind_vector = apparent_wind_vector + local_velocity_vector

        true_wind_speed, true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])
        apparent_wind_speed, apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1])

        global_true_wind_angle = (true_wind_angle + heading) % 360

        desired_position = self.waypoints[self.current_waypoint_index]
        distance_to_desired_position = get_distance_between_positions(current_position, desired_position)

        # HAS THE BOAT REACHED THE WAYPOINT?
        if distance_to_desired_position < self.parameters["waypoint_accuracy"]:
            if len(self.waypoints) <= self.current_waypoint_index + 1:  # Has Reached The Final Waypoint
                self.reset()
                return None, None

            self.current_waypoint_index += 1

        if self.current_state == SailboatStates.NORMAL:
            desired_heading = get_bearing(current_position, desired_position)

            desired_heading, should_tack_condition1 = self._apply_decision_zone_tacking_logic(
                heading, desired_heading, true_wind_angle, apparent_wind_angle, distance_to_desired_position
            )

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

            rudder_angle = self.parameters["rudder_hard_over"] * tack_direction

            if self.parameters["perform_forced_jibe_instead_of_tack"]:
                rudder_angle *= -1

            if abs((heading - self.desired_tacking_angle)) % 360 < self.parameters["tack_tolerance"]:  # if we have finished the tack
                self.current_state = SailboatStates.NORMAL

        else:
            Exception("Unsupported State Transition In run_waypoint_mission_step")

        return sail_angle, rudder_angle



    def get_optimal_sail_angle(self, apparent_wind_angle: float) -> float:
        """
        Given an apparent wind angle, there is generally a best sail angle to point your sail at based on the points of sail diagram,
        and this function computes that. The points of sail diagram can be found here: https://americansailing.com/articles/points-of-sail/
        and all this function does is linearly interpolate between the points described in this chart.

        Args:
            apparent_wind_angle (float): the apparent wind angle measured counter-clockwise from the centerline of the boat in degrees

        Returns:
            float: the optimal sail angle from 0-90 degrees where 0 means the sail is fully pulled in and 90 means that the sail is fully let out
        """

        # 180 means wind pushing you backwards, 90 for the sail means let the sails all the way out
        # these are for close hauled, close reach, beam reach, broad reach and running respectively
        # the angles were estimated from a sailing position diagram and adam should probably take a look and move things around as he sees fit

        sail_positions = self.parameters["sail_lookup_table_sail_positions"]
        wind_angles = self.parameters["sail_lookup_table_wind_angles"]

        left = max(filter(lambda pos: pos <= float(apparent_wind_angle), wind_angles))
        right = min(filter(lambda pos: pos >= float(apparent_wind_angle), wind_angles))

        left = wind_angles.index(left)
        right = wind_angles.index(right)

        sail_angle = 0
        if left == right:
            for i in range(len(sail_positions)):
                if float(apparent_wind_angle) == wind_angles[i]:
                    sail_angle = sail_positions[i]
        else:
            slope = (sail_positions[right] - sail_positions[left]) / (wind_angles[right] - wind_angles[left])
            sail_angle = slope * (float(apparent_wind_angle) - wind_angles[left]) + sail_positions[left]

        return sail_angle

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
            Kp=self.parameters["heading_p_gain"],
            Ki=self.parameters["heading_i_gain"],
            Kd=self.parameters["heading_d_gain"],
            n=self.parameters["heading_n_gain"],
            sample_period=self.parameters["autopilot_refresh_rate"],
        )

        error = get_distance_between_angles(desired_heading, heading)
        rudder_angle = self.rudder_pid_controller(error)
        rudder_angle = np.clip(rudder_angle, self.parameters["min_rudder_angle"], self.parameters["max_rudder_angle"])
        return rudder_angle

    def run_rc_control(self, joystick_left_y: float, joystick_right_x: float) -> tuple[float, float]:
        """
        Formulas used: https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio

        Args:
            joystick_left_y (float): the value of the Y joystick from -100 to 100.
                -100 means the joystick is fully down and 100 means the joystick is fully up.

            joystick_right_x (float): the value of the X joystick from -100 to 100.
                -100 means the joystick is fully to the left and 100 means the joystick is fully to the right.

        Returns:
            tuple[float, float]: the desired (sail_angle, rudder_angle) that the boat should use to sail
        """

        min_sail_angle, max_sail_angle = self.parameters["min_sail_angle"], self.parameters["max_sail_angle"]
        sail_angle = (((joystick_left_y - -100) * (max_sail_angle - min_sail_angle)) / (100 - -100)) + min_sail_angle

        min_rudder_angle, max_rudder_angle = self.parameters["min_rudder_angle"], self.parameters["max_rudder_angle"]
        rudder_angle = (((joystick_right_x - -100) * (max_rudder_angle - min_rudder_angle)) / (100 - -100)) + min_rudder_angle

        return sail_angle, rudder_angle
