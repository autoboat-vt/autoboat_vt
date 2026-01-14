import json
from typing import Any

import numpy as np
import numpy.typing as npt
import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, Int32, String

from autoboat_msgs.msg import RCData, WaypointList

from .autopilot_library.sailboat_autopilot import SailboatAutopilot
from .autopilot_library.utils.constants import CONFIG_DIRECTORY, SailboatAutopilotMode
from .autopilot_library.utils.utils_function_library import cartesian_vector_to_polar, get_bearing
from .autopilot_library.utils.position import Position



class SailboatAutopilotNode(Node):
    """
    The autopilot takes in a bunch of sensor data and waypoints and attempts to traverses through
    the waypoints by continuously publishing to the sail angle and rudder angle topics.

    The main function that you should pay attention to is ``update_ros_topics``, since this is the function that is called periodically on a timer.

    NOTE: All units are in standard SI units and angles are generally measured in degrees unless otherwise specified.
    """

    def __init__(self) -> None:
        super().__init__("sailboat_autopilot")

        self.logger = self.get_logger()

        parameters_path = CONFIG_DIRECTORY / "sailboat_default_parameters.json"
        with open(parameters_path, "r", encoding="utf-8") as parameters_file:
            self.raw_autopilot_parameters: dict[str, dict[str, Any]] = json.load(parameters_file)

        # structured like {parameter_name: parameter_value}
        self.autopilot_parameters: dict[str, Any] = {}
        for param in self.raw_autopilot_parameters:
            self.autopilot_parameters[param] = self.raw_autopilot_parameters[param]["default"]


        self.sailboat_autopilot = SailboatAutopilot(parameters=self.autopilot_parameters, logger=self.logger)

        # Initialize ros2 subscriptions, publishers, and timers
        self.autopilot_refresh_timer = self.create_timer(1 / self.autopilot_parameters["autopilot_refresh_rate"], self.update_ros_topics)
        self.publish_default_autopilot_parameters_timer = self.create_timer(0.1, self.publish_default_autopilot_parameters_timer_callback)

        self.default_autopilot_parameters_publisher = self.create_publisher(String, "/default_autopilot_parameters", 1)
        self.create_subscription(Bool, "/default_autopilot_parameters_acknowledgement", self.default_autopilot_parameters_acknowledgement_callback, 1)

        self.create_subscription(String, "/autopilot_parameters", self.autopilot_parameters_callback, 10)
        self.create_subscription(WaypointList, "/waypoints_list", self.waypoints_list_callback, 10)

        self.create_subscription(NavSatFix, "/position", self.position_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, "/velocity", self.velocity_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, "/heading", self.heading_callback, qos_profile_sensor_data)
        self.create_subscription(Vector3, "/apparent_wind_vector", self.apparent_wind_vector_callback, qos_profile_sensor_data)
        self.create_subscription(RCData, "/rc_data", self.rc_data_callback, qos_profile_sensor_data)

        self.current_waypoint_index_publisher = self.create_publisher(Int32, "/current_waypoint_index", 10)
        self.autopilot_mode_publisher = self.create_publisher(String, "/autopilot_mode", qos_profile_sensor_data)
        self.full_autonomy_maneuver_publisher = self.create_publisher(String, "/full_autonomy_maneuver", qos_profile_sensor_data)
        self.desired_heading_publisher = self.create_publisher(Float32, "/desired_heading", 10)

        self.desired_sail_angle_publisher = self.create_publisher(Float32, "/desired_sail_angle", qos_profile_sensor_data)
        self.desired_rudder_angle_publisher = self.create_publisher(Float32, "/desired_rudder_angle", qos_profile_sensor_data)

        self.zero_rudder_encoder_publisher = self.create_publisher(Bool, "/zero_rudder_encoder", 10)
        self.zero_winch_encoder_publisher = self.create_publisher(Bool, "/zero_winch_encoder", 10)

        # Default values
        self.position = Position(longitude=0.0, latitude=0.0)
        self.global_velocity: npt.NDArray[np.float64] = np.zeros(2, dtype=np.float64)
        self.speed: float = 0.0
        self.heading: float = 0.0
        self.apparent_wind_vector: npt.NDArray[np.float64] = np.zeros(2, dtype=np.float64)
        self.apparent_wind_vector: npt.NDArray[np.float64] = np.zeros(2, dtype=np.float64)
        self.apparent_wind_angle: float = 0.0
        self.sail_angle: float = 0.0
        self.rudder_angle: float = 0.0

        self.has_default_autopilot_parameters_been_received_by_telemetry_node = False

        self.should_zero_rudder_encoder = False
        self.rudder_encoder_has_been_zeroed = False

        self.should_zero_winch_encoder = False
        self.winch_encoder_has_been_zeroed = False

        # Should this be by default: SailboatAutopilotMode.Full_RC
        self.autopilot_mode: SailboatAutopilotMode = SailboatAutopilotMode.WAYPOINT_MISSION
        self.autopilot_mode: SailboatAutopilotMode = SailboatAutopilotMode.WAYPOINT_MISSION

        self.heading_to_hold: float = 0.0

        self.joystick_left_x: float = 0.0
        self.joystick_left_y: float = 0.0
        self.joystick_right_x: float = 0.0
        self.joystick_right_y: float = 0.0

        self.button_a: bool = False
        self.toggle_b: int = 0
        self.toggle_c: int = 0
        self.button_d: bool = False
        self.toggle_e: int = 0
        self.toggle_f: int = 0



    def rc_data_callback(self, rc_data_message: RCData) -> None:
        """
        This callback is called whenever there is new data about what is being pressed on the remote control.
        Whenever this callback is called, a couple of things happen: first, we check whether or not we are trying to zero
        the rudder or the winch and if we have, then we should handle that. Second, we need to keep track of if we are entering
        hold heading mode, and if we are, then we need to keep track of the current heading, since that heading is the one we will need to hold.
        Finally, we set the SailboatAutopilotMode based on a specific combination of toggles to determine whether we are in full rc mode,
        full autopilot mode, or one of the semi autonomous modes.

        Args:
            rc_data_message (RCData): A struct that contains all of the data on what is pressed on the remote control
        """

        # This means we have entered hold heading mode, so keep track of the current heading since this is the target heading
        if rc_data_message.toggle_f == 1 and self.toggle_f != 1:
            self.heading_to_hold = self.heading

        # Are we trying to zero the rudder?
        if self.button_d is False and rc_data_message.button_d is True:
            self.should_zero_rudder_encoder = True
            self.rudder_encoder_has_been_zeroed = False

        elif self.rudder_encoder_has_been_zeroed:
            self.should_zero_rudder_encoder = False

        # Are we trying to zero the winch?
        if self.button_a is False and rc_data_message.button_a is True:
            self.should_zero_winch_encoder = True
            self.winch_encoder_has_been_zeroed = False

        elif self.winch_encoder_has_been_zeroed:
            self.should_zero_winch_encoder = False

        self.joystick_left_x = rc_data_message.joystick_left_x
        self.joystick_left_y = rc_data_message.joystick_left_y
        self.joystick_right_x = rc_data_message.joystick_right_x
        self.joystick_right_y = rc_data_message.joystick_right_y

        self.button_a = rc_data_message.button_a
        self.toggle_b = rc_data_message.toggle_b
        self.toggle_c = rc_data_message.toggle_c
        self.button_d = rc_data_message.button_d
        self.toggle_e = rc_data_message.toggle_e
        self.toggle_f = rc_data_message.toggle_f

        # kill switch
        if self.toggle_b != 0:
            self.autopilot_mode = SailboatAutopilotMode.DISABLED

        # full autonomy
        elif self.toggle_f == 2:
            self.autopilot_mode = SailboatAutopilotMode.WAYPOINT_MISSION

        # hold heading to the direction that we started this mode in. the sail is controlled via RC
        elif self.toggle_f == 1 and self.toggle_c == 0:
            self.autopilot_mode = SailboatAutopilotMode.HOLD_HEADING

        # choose the best sail angle based on the lookup table. the rudder is controlled via RC
        elif self.toggle_f == 1 and self.toggle_c == 1:
            self.autopilot_mode = SailboatAutopilotMode.HOLD_BEST_SAIL

        # hold heading and best sail
        elif self.toggle_f == 1 and self.toggle_c == 2:
            self.autopilot_mode = SailboatAutopilotMode.HOLD_HEADING_AND_BEST_SAIL

        # remote controlled
        elif self.toggle_f == 0:
            self.autopilot_mode = SailboatAutopilotMode.FULL_RC

        # this should never happen
        else:
            print("WARNING: INCORRECT COMBINATION OF RC SWITCHES USED")


    def autopilot_parameters_callback(self, new_parameters: String) -> None:
        """
        Receives a serialized json (as a string) of parameters and sets them as constants.
        Any constant can be set as long as they are in the json
        """

        new_parameters_json: dict[str, Any] = json.loads(new_parameters.data)
        
        for new_parameter_name, new_parameter_value in new_parameters_json.items():
            
            if new_parameter_name not in self.autopilot_parameters:
                warn_string = "WARNING: Attempted to set an autopilot parameter that the autopilot doesn't know. "
                warn_string += "If you would like to make a new autopilot parameter, please edit sailboat_default_parameters.json"
                self.get_logger().warn(warn_string)
                continue

            self.autopilot_parameters[new_parameter_name] = new_parameter_value


        # HANDLE SPECIAL CASES SINCE THEY DO NOT UPDATE AUTOMATICALLY
        if "autopilot_refresh_rate" in new_parameters_json:
            self.destroy_timer(self.autopilot_refresh_timer)
            
            self.autopilot_refresh_timer = self.create_timer(
                timer_period_sec = 1/self.autopilot_parameters["autopilot_refresh_rate"], 
                callback = self.update_ros_topics
            )



    def waypoints_list_callback(self, waypoint_list: WaypointList) -> None:
        """
        Convert the list of ``NavSatFix`` objects (ROS2) to a list of ``Position`` objects, 
        which are a custom datatype that has some useful helper methods.

        References
        ----------
        ``NavSatFix`` message documentation: https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html
        """

        if len(waypoint_list.waypoints) == 0:
            return

        self.sailboat_autopilot.reset()

        waypoint_navsatfixes: list[NavSatFix] = waypoint_list.waypoints
        
        waypoint_positions: list[Position] = []
        for waypoint in waypoint_navsatfixes:
            waypoint_positions.append(Position(longitude=waypoint.longitude, latitude=waypoint.latitude))
        
        self.sailboat_autopilot.update_waypoints_list(waypoint_positions)

    
    def default_autopilot_parameters_acknowledgement_callback(self, default_autopilot_parameters_acknowledgement: Bool) -> None:
        self.has_default_autopilot_parameters_been_received_by_telemetry_node = default_autopilot_parameters_acknowledgement.data


    def position_callback(self, position: NavSatFix) -> None:
        self.position = Position(longitude=position.longitude, latitude=position.latitude)


    def velocity_callback(self, global_velocity: Twist) -> None:
        self.global_velocity = np.array([global_velocity.linear.x, global_velocity.linear.y])
        self.global_velocity = np.array([global_velocity.linear.x, global_velocity.linear.y])
        self.speed = np.linalg.norm(self.global_velocity)


    def heading_callback(self, heading: Float32) -> None:
        self.heading = heading.data


    def apparent_wind_vector_callback(self, apparent_wind_vector: Vector3) -> None:
        self.apparent_wind_vector = np.array([apparent_wind_vector.x, apparent_wind_vector.y])
        _, self.apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector.x, apparent_wind_vector.y)


    def publish_default_autopilot_parameters_timer_callback(self) -> None:
        """
        Publish the default parameters so that the telemetry node/telemetry server/groundstation
        know which parameters it can change. This should only be sent to the telemetry node once
        and then should never be received again once the telemetry node properly receives and parses it.
        """

        if not self.has_default_autopilot_parameters_been_received_by_telemetry_node:
            self.default_autopilot_parameters_publisher.publish(String(data=json.dumps(self.autopilot_parameters)))



    def step(self) -> tuple[float | None, float | None]:
        """
        TODO perhaps in the future, make this function state independent 
        (aka using no self.position, self.global_velocity etc and just having them passed in as arguments)
        
        Computes the best sail and rudder angles for the current mode and state

        Returns (tuple): (sail_angle, rudder_angle)
            sail angle or rudder angle are None if the autopilot doesn't have authority over them
        """

        sail_angle: float | None = None
        rudder_angle: float | None = None

        if self.autopilot_mode == SailboatAutopilotMode.WAYPOINT_MISSION and self.sailboat_autopilot.waypoints is not None:
            sail_angle, rudder_angle = self.sailboat_autopilot.run_waypoint_mission_step(
                self.position,
                self.global_velocity,
                self.heading,
                self.apparent_wind_vector
            )

        elif self.autopilot_mode == SailboatAutopilotMode.HOLD_BEST_SAIL:
            sail_angle = self.sailboat_autopilot.get_optimal_sail_angle(self.apparent_wind_angle)
            _, rudder_angle = self.sailboat_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)
            sail_angle = self.sailboat_autopilot.get_optimal_sail_angle(self.apparent_wind_angle)
            _, rudder_angle = self.sailboat_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)

        elif self.autopilot_mode == SailboatAutopilotMode.HOLD_HEADING:
            rudder_angle = self.sailboat_autopilot.get_optimal_rudder_angle(self.heading, self.heading_to_hold)
            sail_angle, _ = self.sailboat_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)
            rudder_angle = self.sailboat_autopilot.get_optimal_rudder_angle(self.heading, self.heading_to_hold)
            sail_angle, _ = self.sailboat_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)

        elif self.autopilot_mode == SailboatAutopilotMode.HOLD_HEADING_AND_BEST_SAIL:
            rudder_angle = self.sailboat_autopilot.get_optimal_rudder_angle(self.heading, self.heading_to_hold)
            sail_angle = self.sailboat_autopilot.get_optimal_sail_angle(self.apparent_wind_angle)
            rudder_angle = self.sailboat_autopilot.get_optimal_rudder_angle(self.heading, self.heading_to_hold)
            sail_angle = self.sailboat_autopilot.get_optimal_sail_angle(self.apparent_wind_angle)

        elif self.autopilot_mode == SailboatAutopilotMode.FULL_RC:
            sail_angle, rudder_angle = self.sailboat_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)

        return sail_angle, rudder_angle



    def update_ros_topics(self) -> None:
        """
        This is the main function that is called constantely by the timer.
        Updates the sail_angle and rudder_angle topics based on the output of stepping in the autopilot controller
        """

        desired_sail_angle, desired_rudder_angle = self.step()

        self.current_waypoint_index_publisher.publish(Int32(data=self.sailboat_autopilot.current_waypoint_index))

        # Publish the autonomy maneuever (aka whether we are currently CW tacking, CCW tacking, or normal sailing)
        self.autopilot_mode_publisher.publish(String(data=self.autopilot_mode.name))
        if self.autopilot_mode == SailboatAutopilotMode.WAYPOINT_MISSION:
            self.full_autonomy_maneuver_publisher.publish(String(data=self.sailboat_autopilot.current_state.name))

        else:
            self.full_autonomy_maneuver_publisher.publish(String(data="N/A"))


        # Publish the desired heading
        if (self.autopilot_mode == SailboatAutopilotMode.HOLD_HEADING or self.autopilot_mode == SailboatAutopilotMode.HOLD_HEADING_AND_BEST_SAIL):
            self.desired_heading_publisher.publish(Float32(data=float(self.heading_to_hold)))

        elif (self.autopilot_mode == SailboatAutopilotMode.WAYPOINT_MISSION and self.sailboat_autopilot.waypoints is not None):
            current_waypoint = self.sailboat_autopilot.waypoints[self.sailboat_autopilot.current_waypoint_index]

            # TODO make it so that the bearing is the actual heading the autopilot is trying to follow (this is different when tacking)
            # when tacking, the boat is not trying to head straight towards the waypoint, but rather, it is travelling on a tacking line
            # maybe add a new ROS topic specifically for the actual heading that the autopilot is trying to follow
            bearing_to_waypoint = get_bearing(self.position, current_waypoint)
            self.desired_heading_publisher.publish(Float32(data=float(bearing_to_waypoint)))

        else:
            self.desired_heading_publisher.publish(Float32(data=0.0))



        # Ensure that we tell the motor driver what we want the rudder angle and the sail angle to do through ros
        if desired_rudder_angle is not None:
            self.desired_rudder_angle_publisher.publish(Float32(data=float(desired_rudder_angle)))  # the negative is a correction for how to actually turn the boat

        if desired_sail_angle is not None:
            self.desired_sail_angle_publisher.publish(Float32(data=float(desired_sail_angle)))

        if self.should_zero_rudder_encoder:
            self.zero_rudder_encoder_publisher.publish(Bool(data=self.should_zero_rudder_encoder))
            self.rudder_encoder_has_been_zeroed = True

        if self.should_zero_winch_encoder:
            self.zero_winch_encoder_publisher.publish(Bool(data=self.should_zero_winch_encoder))
            self.winch_encoder_has_been_zeroed = True





def main() -> None:
    rclpy.init()
    sailboat_autopilot_node = SailboatAutopilotNode()
    rclpy.spin(sailboat_autopilot_node)

    sailboat_autopilot_node.destroy_node()
    rclpy.shutdown()
