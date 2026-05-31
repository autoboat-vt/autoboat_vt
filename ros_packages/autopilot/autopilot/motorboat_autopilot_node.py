import json
import time
from typing import Any

import numpy as np
import numpy.typing as npt
import rclpy
from autoboat_msgs.msg import RCData, VESCControlData, WaypointList
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, Int32, String

from .autopilot_library.motorboat_autopilot import MotorboatAutopilot
from .autopilot_library.utils.constants import (
    CONFIG_DIRECTORY,
    QOS_AUTOPILOT_PARAMETER_CONFIG_PATH,
    MotorboatControlModes,
    PropellerMotorControlMode,
)
from .autopilot_library.utils.position import Position
from .autopilot_library.utils.utils_function_library import get_bearing


class MotorboatAutopilotNode(Node):
    """
    The autopilot takes in a bunch of sensor data and waypoints and attempts to traverses through
    the waypoints by continuously publishing to the propeller motor control struct and rudder angle topics.

    The main function that you should pay attention to is ``update_ros_topics``,
    since this is the function that is called periodically on a timer.

    NOTE: All units are in standard SI units and angles are generally measured in degrees unless otherwise specified.
    """

    def __init__(self) -> None:
        super().__init__("motorboat_autopilot")

        self.logger = self.get_logger()

        self.autopilot_parameter_config_path_publisher = self.create_publisher(
            String, "/autopilot_parameter_config_path", QOS_AUTOPILOT_PARAMETER_CONFIG_PATH
        )
        parameters_path = CONFIG_DIRECTORY / "motorboat_default_parameters.json"
        self.autopilot_parameter_config_path_publisher.publish(String(data=parameters_path.as_posix()))

        with open(parameters_path, "r", encoding="utf-8") as parameters_file:
            self.raw_autopilot_parameters: dict[str, dict[str, Any]] = json.load(parameters_file)

        # structured like {parameter_name: parameter_value}
        self.autopilot_parameters: dict[str, Any] = {}
        for param in self.raw_autopilot_parameters:
            self.autopilot_parameters[param] = self.raw_autopilot_parameters[param]["default"]


        self.motorboat_autopilot = MotorboatAutopilot(parameters=self.autopilot_parameters, logger=self.get_logger())

        # Initialize ROS2 subscriptions, publishers, and timers
        autopilot_refresh_period = 1 / self.autopilot_parameters["autopilot_refresh_rate"]
        self.autopilot_refresh_timer = self.create_timer(autopilot_refresh_period, self.update_ros_topics)

        self.create_subscription(String, "/autopilot_parameters", self.autopilot_parameters_callback, 10)
        self.create_subscription(WaypointList, "/waypoints_list", self.waypoints_list_callback, 10)
        self.create_subscription(NavSatFix, "/position", self.position_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, "/velocity", self.velocity_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, "/heading", self.heading_callback, qos_profile_sensor_data)
        self.create_subscription(RCData, "/rc_data", self.rc_data_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, "/object_detection_emergency_stop", self.emergency_stop_callback, 10)

        self.current_waypoint_index_publisher = self.create_publisher(Int32, "/current_waypoint_index", 10)
        self.boat_control_mode_publisher = self.create_publisher(String, "/boat_control_mode", qos_profile_sensor_data)
        self.desired_heading_publisher = self.create_publisher(Float32, "/desired_heading", 10)

        self.should_propeller_motor_be_powered_publisher = self.create_publisher(Bool, "/should_propeller_motor_be_powered", 10)
        self.propeller_motor_control_struct_publisher = self.create_publisher(
            VESCControlData, "/propeller_motor_control_struct", qos_profile_sensor_data
        )

        self.desired_rudder_angle_publisher = self.create_publisher(Float32, "/desired_rudder_angle", 10)
        self.zero_rudder_encoder_publisher = self.create_publisher(Bool, "/zero_rudder_encoder", 10)





        # default values
        self.position = Position(longitude=0.0,latitude=0.0)
        self.velocity: npt.NDArray[np.float64] = np.zeros(2, dtype=np.float64)
        self.speed = 0.0
        self.heading = 0.0
        self.rudder_angle = 0.0

        self.motorboat_control_mode = MotorboatControlModes.WAYPOINT_MISSION
        self.propeller_motor_control_mode = PropellerMotorControlMode.RPM
        self.should_propeller_motor_be_powered = False
        self.should_zero_encoder = False
        self.encoder_has_been_zeroed = False
        self.heading_to_hold = 0.0
        self.has_default_autopilot_parameters_been_received_by_telemetry_node = False

        # used to check whether we have disconnected from any of the sensors and stop the propeller
        self.last_rc_data_received_time: float = 0.0
        self.last_gps_position_received_time: float = 0.0
        self.last_heading_received_time: float = 0.0

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
        Whenever this callback is called, a couple of things happen:
        1. We check whether or not we are trying to zero the rudder and if we have, then we should handle that.

        2. We need to keep track of if we are entering hold heading mode, and if we are,
        then we need to keep track of the current heading, since that heading is the one we will need to hold.

        3. We set the SailboatControlMode based on a specific combination of toggles to determine whether we are in
        full rc mode, full autopilot mode, or one of the semi autonomous modes.
        Finally, we need to set the propeller control mode based on the toggles to determine whether we are controlling
        the propeller based on current output, duty cycle output, or propeller RPM.

        Parameters
        ----------
            rc_data_message (RCData): A struct that contains all of the data on what is pressed on the remote control
        """

        self.last_rc_data_received_time = time.time()

        # This means we have entered hold heading mode, so keep track of the current heading since this is the target heading
        if rc_data_message.toggle_f == 1 and self.toggle_f != 1:
            self.heading_to_hold = self.heading

        # Are we trying to zero the rudder?
        if self.button_d is False and rc_data_message.button_d is True:
            self.should_zero_encoder = True
            self.encoder_has_been_zeroed = False

        elif self.encoder_has_been_zeroed:
            self.should_zero_encoder = False

        self.joystick_left_x = rc_data_message.joystick_left_x

        # this is negated because positive throttle ends up making the boat go in reverse
        self.joystick_left_y = -1 * rc_data_message.joystick_left_y

        self.joystick_right_x = rc_data_message.joystick_right_x
        self.joystick_right_y = rc_data_message.joystick_right_y

        self.button_a = rc_data_message.button_a
        self.toggle_b = rc_data_message.toggle_b
        self.toggle_c = rc_data_message.toggle_c
        self.button_d = rc_data_message.button_d
        self.toggle_e = rc_data_message.toggle_e
        self.toggle_f = rc_data_message.toggle_f

        # kill switch
        if self.toggle_b == 0:
            self.should_propeller_motor_be_powered = True
        elif self.toggle_b == 1:
            self.should_propeller_motor_be_powered = False

        if self.toggle_b == 2:
            self.motorboat_control_mode = MotorboatControlModes.DISABLED

        # full autonomy
        elif self.toggle_f == 2:
            self.motorboat_control_mode = MotorboatControlModes.WAYPOINT_MISSION

        # hold heading to the direction that we started this mode in
        elif self.toggle_f == 1:
            self.motorboat_control_mode = MotorboatControlModes.HOLD_HEADING

        # remote controlled
        elif self.toggle_f == 0:
            self.motorboat_control_mode = MotorboatControlModes.FULL_RC
        # should not happen
        else:
            print("WARNING: INCORRECT COMBINATION OF RC SWITCHES USED")


        # Check the type of control that is selected
        if self.toggle_c == 0:
            self.propeller_motor_control_mode = PropellerMotorControlMode.RPM

        elif self.toggle_c == 1:
            self.propeller_motor_control_mode = PropellerMotorControlMode.DUTY_CYCLE

        elif self.toggle_c == 2:
            self.propeller_motor_control_mode = PropellerMotorControlMode.CURRENT




    def autopilot_parameters_callback(self, new_parameters: String) -> None:
        """
        Updates the autopilot parameters based on the input JSON string.

        NOTE: Only parameters that already exist in the autopilot will be updated.

        Parameters
        ----------
        new_parameters
            A JSON string that contains the new parameters to set for the autopilot.
        """

        new_parameters_json: dict = json.loads(new_parameters.data)

        for new_parameter_name, new_parameter_value in new_parameters_json.items():
            if new_parameter_name not in self.autopilot_parameters:
                print("WARNING: Attempted to set an autopilot parameter that the autopilot doesn't know")
                print("If you would like to make a new autopilot parameter, please edit motorboat_default_parameters.json")
                continue

            self.autopilot_parameters[new_parameter_name] = new_parameter_value

        # special cases to handle since they do not update automatically
        if "autopilot_refresh_rate" in new_parameters_json:
            self.destroy_timer(self.autopilot_refresh_timer)
            self.autopilot_refresh_timer = self.create_timer(
                1 / self.autopilot_parameters["autopilot_refresh_rate"], self.update_ros_topics
            )


    def waypoints_list_callback(self, waypoint_list: WaypointList) -> None:
        """
        Callback function that is called whenever there is a new waypoint list message.
        Converts the list of ROS2 ``NavSatFix`` objects to a list of ``Position`` objects,
        which are a custom datatype that has some useful helper methods.

        Does not directly set the waypoints in the motorboat autopilot object,
        instead it calls the ``update_waypoints_list`` method of the motorboat autopilot object.

        Parameters
        ----------
        waypoint_list
            A ROS2 message that contains a list of waypoints as ``NavSatFix`` objects.
        """

        if len(waypoint_list.waypoints) == 0:
            return

        self.motorboat_autopilot.reset()

        waypoint_navsatfixes: list[NavSatFix] = waypoint_list.waypoints

        waypoint_positions: list[Position] = []
        for waypoint in waypoint_navsatfixes:
            waypoint_positions.append(Position(longitude=waypoint.longitude, latitude=waypoint.latitude))

        self.motorboat_autopilot.update_waypoints_list(waypoint_positions)


    def position_callback(self, position: NavSatFix) -> None:
        """A callback function to get the current position of the boat."""
        self.position = Position(longitude=position.longitude, latitude=position.latitude)
        self.last_gps_position_received_time = time.time()


    def velocity_callback(self, velocity: Twist) -> None:
        """A callback function to get the current velocity and speed of the boat."""
        self.velocity = np.array([velocity.linear.x, velocity.linear.y], dtype=np.float64)
        self.speed = np.linalg.norm(self.velocity)


    def heading_callback(self, heading: Float32) -> None:
        """A callback function to get the current heading of the boat."""
        self.heading = heading.data
        self.last_heading_received_time = time.time()

    def emergency_stop_callback(self, should_emergency_stop: Bool)-> None:
        """
        A callback function for the /object_detection_emergency_stop topic.
        If this topic has a 'true' in it and we are on a waypoint mission,
        then we need to emergency stop the boat until the emergency stop is
        turned to 'false'.
        """
        if should_emergency_stop.data and self.motorboat_control_mode == MotorboatControlModes.WAYPOINT_MISSION:
            self.motorboat_control_mode = MotorboatControlModes.EMERGENCY_STOP

        elif not should_emergency_stop.data:
            self.motorboat_control_mode = MotorboatControlModes.WAYPOINT_MISSION




    def update_ros_topics(self) -> None:
        """
        This is the main function that is called constantely by the timer.
        Updates the propeller control struct and rudder angle topics based on the output of the autopilot controller.
        """

        desired_rudder_angle = 0.0
        desired_vesc_control_struct = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)


        # Manually check whether any of the sensors have disconnecteds
        # if we have not received data from the sensor for 3 seconds then stop the propeller for safety reasons
        time_since_last_rc_data = time.time() - self.last_rc_data_received_time
        has_rc_data_disconnected = time_since_last_rc_data >= self.autopilot_parameters["rc_data_failsafe_time"]

        time_since_last_gps_data = time.time() - self.last_gps_position_received_time
        has_gps_disconnected = time_since_last_gps_data >= self.autopilot_parameters["gps_position_failsafe_time"]

        time_since_last_heading_data = time.time() - self.last_heading_received_time
        has_heading_disconnected = time_since_last_heading_data >= self.autopilot_parameters["heading_data_failsafe_time"]

        has_sensor_disconnected = has_rc_data_disconnected or has_gps_disconnected or has_heading_disconnected


        if has_sensor_disconnected:
            self.get_logger().info(f"has sensor disconnected: {has_sensor_disconnected}")
            self.get_logger().info(f"has remote controller disconnected: {has_rc_data_disconnected}")
            self.get_logger().info(f"has gps disconnected: {has_gps_disconnected}")
            self.get_logger().info(f"has heading disconnected: {has_heading_disconnected}")

            # Set the values to 0 as a failsafe so that the boat doesn't make bad decisions
            # while it lost connection with some of its sensors
            desired_vesc_control_struct = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)

        elif self.motorboat_control_mode == MotorboatControlModes.EMERGENCY_STOP:
            desired_vesc_control_struct = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)
            desired_rudder_angle = self.autopilot_parameters["rudder_hard_over"]

        elif self.motorboat_control_mode == MotorboatControlModes.WAYPOINT_MISSION and self.motorboat_autopilot.waypoints is not None:
            desired_rpm, desired_rudder_angle = self.motorboat_autopilot.run_waypoint_mission_step(self.position, self.heading)
            desired_vesc_control_struct = VESCControlData(control_type_for_vesc="rpm", control_value=desired_rpm)

        elif self.motorboat_control_mode == MotorboatControlModes.HOLD_HEADING:
            desired_rudder_angle = self.motorboat_autopilot.get_optimal_rudder_angle(self.heading, self.heading_to_hold)
            desired_vesc_control_type, desired_vesc_control_value, _ = self.motorboat_autopilot.run_rc_control(
                self.joystick_left_y, self.joystick_right_x, self.propeller_motor_control_mode
            )
            desired_vesc_control_struct = VESCControlData(
                control_type_for_vesc=desired_vesc_control_type, control_value=desired_vesc_control_value
            )

        elif self.motorboat_control_mode == MotorboatControlModes.FULL_RC:
            desired_vesc_control_type, desired_vesc_control_value, desired_rudder_angle = self.motorboat_autopilot.run_rc_control(
                self.joystick_left_y, self.joystick_right_x, self.propeller_motor_control_mode
            )
            desired_vesc_control_struct = VESCControlData(
                control_type_for_vesc=desired_vesc_control_type, control_value=desired_vesc_control_value
            )

        else:
            desired_vesc_control_struct = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)


        # Now that we are done computing what the autopilot should do, publish everything to their respective topics

        self.propeller_motor_control_struct_publisher.publish(desired_vesc_control_struct)

        if desired_rudder_angle is not None:
            self.desired_rudder_angle_publisher.publish(Float32(data=float(desired_rudder_angle)))

        if self.should_zero_encoder:
            self.zero_rudder_encoder_publisher.publish(Bool(data=self.should_zero_encoder))
            self.encoder_has_been_zeroed = True

        self.should_propeller_motor_be_powered_publisher.publish(Bool(data=self.should_propeller_motor_be_powered))


        self.current_waypoint_index_publisher.publish(Int32(data=self.motorboat_autopilot.current_waypoint_index))

        self.boat_control_mode_publisher.publish(String(data=self.motorboat_control_mode.name))

        if self.motorboat_control_mode == MotorboatControlModes.HOLD_HEADING:
            self.desired_heading_publisher.publish(Float32(data=float(self.heading_to_hold)))

        elif self.motorboat_control_mode == MotorboatControlModes.WAYPOINT_MISSION and self.motorboat_autopilot.waypoints is not None:
            current_waypoint = self.motorboat_autopilot.waypoints[self.motorboat_autopilot.current_waypoint_index]
            bearing_to_waypoint = get_bearing(self.position, current_waypoint)
            self.desired_heading_publisher.publish(Float32(data=float(bearing_to_waypoint)))
        else:
            self.desired_heading_publisher.publish(Float32(data=0.))




def main() -> None:
    rclpy.init()
    motorboat_autopilot_node = MotorboatAutopilotNode()
    rclpy.spin(motorboat_autopilot_node)

    motorboat_autopilot_node.destroy_node()
    rclpy.shutdown()
