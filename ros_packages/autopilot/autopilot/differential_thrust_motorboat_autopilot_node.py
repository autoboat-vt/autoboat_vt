import json
import time
from typing import Any

import numpy as np
import numpy.typing as npt

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, Int32, String

from autoboat_msgs.msg import RCData, VESCControlData, WaypointList

from .autopilot_library.differential_thrust_motorboat_autopilot import DifferentialThrustMotorboatAutopilot
from .autopilot_library.utils.constants import (
    CONFIG_DIRECTORY,
    QOS_AUTOPILOT_PARAMETER_CONFIG_PATH,
    MotorboatControlModes,
    PropellerMotorControlMode,
)
from .autopilot_library.utils.position import Position
from .autopilot_library.utils.utils_function_library import get_bearing


class DifferentialThrustMotorboatAutopilotNode(Node):
    """
    ROS2 node adapter for the differential-thrust motorboat autopilot.

    This node follows the same layout and callback flow as the existing
    motorboat node, but it publishes two VESC control messages: one for the
    left motor and one for the right motor.
    """

    def __init__(self) -> None:
        # Create the ROS node itself. This node follows the same lifecycle pattern
        # as the original motorboat autopilot node, but the actuator output is
        # split into left and right differential-thrust commands.
        super().__init__("differential_thrust_motorboat_autopilot")

        self.logger = self.get_logger()

        # Publish the configuration file location so telemetry and external tools
        # know which parameter set this autopilot is using.
        self.autopilot_parameter_config_path_publisher = self.create_publisher(
            String, "/autopilot_parameter_config_path", QOS_AUTOPILOT_PARAMETER_CONFIG_PATH
        )
        parameters_path = CONFIG_DIRECTORY / "differential_thrust_motorboat_default_parameters.json"
        self.autopilot_parameter_config_path_publisher.publish(String(data=parameters_path.as_posix()))

        with open(parameters_path, "r", encoding="utf-8") as parameters_file:
            self.raw_autopilot_parameters: dict[str, dict[str, Any]] = json.load(parameters_file)

        # Load the autopilot parameter dictionary. The values here are the same
        # ones used by the original motorboat controller, but they are consumed by
        # the differential-thrust autopilot logic in a left/right motor split.
        self.autopilot_parameters: dict[str, Any] = {}
        for param in self.raw_autopilot_parameters:
            self.autopilot_parameters[param] = self.raw_autopilot_parameters[param]["default"]

        # Instantiate the controller logic. This keeps the math inside the
        # autopilot library and leaves the ROS node as an adapter layer that only
        # handles subscriptions, mode selection, and publishing.
        self.differential_thrust_motorboat_autopilot = DifferentialThrustMotorboatAutopilot(
            parameters=self.autopilot_parameters,
            logger=self.get_logger(),
        )

        # This timer is the main control loop. It periodically decides what the
        # boat should do based on mode, sensor health, and the current goal.
        autopilot_refresh_period = 1 / self.autopilot_parameters["autopilot_refresh_rate"]
        self.autopilot_refresh_timer = self.create_timer(autopilot_refresh_period, self.update_ros_topics)

        # Subscribe to the sensor and mission data needed for the boat to make
        # decisions. This mirrors the behavior of the original motorboat node but
        # passes the outputs through the differential-thrust controller.
        self.create_subscription(String, "/autopilot_parameters", self.autopilot_parameters_callback, 10)
        self.create_subscription(WaypointList, "/waypoints_list", self.waypoints_list_callback, 10)
        self.create_subscription(NavSatFix, "/position", self.position_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, "/velocity", self.velocity_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, "/heading", self.heading_callback, qos_profile_sensor_data)
        self.create_subscription(RCData, "/rc_data", self.rc_data_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, "/object_detection_emergency_stop", self.emergency_stop_callback, 10)

        # Publish mission and system state so the rest of the stack can monitor the
        # boat state even though the motor outputs are now differential.
        self.current_waypoint_index_publisher = self.create_publisher(Int32, "/current_waypoint_index", 10)
        self.boat_control_mode_publisher = self.create_publisher(String, "/boat_control_mode", qos_profile_sensor_data)
        self.desired_heading_publisher = self.create_publisher(Float32, "/desired_heading", 10)

        # The propulsion system for this boat has two independent motor channels,
        # so each side gets its own VESC command message instead of a single
        # combined propeller struct.
        self.should_propeller_motor_be_powered_publisher = self.create_publisher(Bool, "/should_propeller_motor_be_powered", 10)
        self.left_propeller_motor_control_struct_publisher = self.create_publisher(
            VESCControlData,
            "/left_propeller_motor_control_struct",
            qos_profile_sensor_data,
        )
        self.right_propeller_motor_control_struct_publisher = self.create_publisher(
            VESCControlData,
            "/right_propeller_motor_control_struct",
            qos_profile_sensor_data,
        )

        # The node continues to expose a heading publisher and status flags in the
        # same format as the motorboat node for the rest of the autopilot stack.
        self.desired_rudder_angle_publisher = self.create_publisher(Float32, "/desired_rudder_angle", 10)
        self.zero_rudder_encoder_publisher = self.create_publisher(Bool, "/zero_rudder_encoder", 10)

        self.position = Position(longitude=0.0, latitude=0.0)
        self.velocity: npt.NDArray[np.float64] = np.zeros(2, dtype=np.float64)
        self.speed = 0.0
        self.heading = 0.0
        self.rudder_angle = 0.0

        self.motorboat_control_mode = MotorboatControlModes.WAYPOINT_MISSION
        self.propeller_motor_control_mode = PropellerMotorControlMode.RPM
        self.should_propeller_motor_be_powered = True
        self.emergency_stop_active = False
        self.should_zero_encoder = False
        self.encoder_has_been_zeroed = False
        self.heading_to_hold = 0.0
        self.has_default_autopilot_parameters_been_received_by_telemetry_node = False

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
        Handle incoming remote-control data.

        This callback updates the current RC state, decides the active boat mode,
        and tracks the kill switch and propeller control mode selection. The
        logical flow mirrors the original motorboat node, but the resulting
        command is distributed across the left and right motors instead of a
        single rudder-based output.

        Parameters
        ----------
        rc_data_message
            Message containing joystick values, switch states, and buttons from
            the RC transmitter.
        """
        # Record the latest RC update time and use it to detect lost RC input.
        self.last_rc_data_received_time = time.time()

        # When the user enters hold-heading mode, capture the current heading and
        # use it as the target heading for the controller to maintain.
        if rc_data_message.toggle_f == 1 and self.toggle_f != 1:
            self.heading_to_hold = self.heading

        # Handle rudder encoder zeroing, which is retained to match the rest of
        # the boat control system even though the actual steering is performed by
        # differential thrust instead of a physical rudder.
        if self.button_d is False and rc_data_message.button_d is True:
            self.should_zero_encoder = True
            self.encoder_has_been_zeroed = False
        elif self.encoder_has_been_zeroed:
            self.should_zero_encoder = False

        # Store the joystick values. The left vertical axis still represents the
        # throttle command, while the right horizontal axis becomes the steering
        # bias used to split the left/right motor commands.
        self.joystick_left_x = rc_data_message.joystick_left_x
        self.joystick_left_y = -1 * rc_data_message.joystick_left_y
        self.joystick_right_x = rc_data_message.joystick_right_x
        self.joystick_right_y = rc_data_message.joystick_right_y

        self.button_a = rc_data_message.button_a
        self.toggle_b = rc_data_message.toggle_b
        self.toggle_c = rc_data_message.toggle_c
        self.button_d = rc_data_message.button_d
        self.toggle_e = rc_data_message.toggle_e
        self.toggle_f = rc_data_message.toggle_f

        # The kill switch still controls whether the propulsion system is allowed
        # to generate thrust. The remaining switch logic still sets the boat mode
        # in the same way as the original node, but the actuator output is now a
        # left/right differential pair instead of a rudder-driven single output.
        if self.toggle_b == 0:
            self.should_propeller_motor_be_powered = True
        elif self.toggle_b == 1:
            self.should_propeller_motor_be_powered = False

        if self.toggle_b == 2:
            self.motorboat_control_mode = MotorboatControlModes.DISABLED
        elif self.toggle_f == 2:
            self.motorboat_control_mode = MotorboatControlModes.WAYPOINT_MISSION
        elif self.toggle_f == 1:
            self.motorboat_control_mode = MotorboatControlModes.HOLD_HEADING
        elif self.toggle_f == 0:
            self.motorboat_control_mode = MotorboatControlModes.FULL_RC
        else:
            print("WARNING: INCORRECT COMBINATION OF RC SWITCHES USED")

        if self.toggle_c == 0:
            self.propeller_motor_control_mode = PropellerMotorControlMode.RPM
        elif self.toggle_c == 1:
            self.propeller_motor_control_mode = PropellerMotorControlMode.DUTY_CYCLE
        elif self.toggle_c == 2:
            self.propeller_motor_control_mode = PropellerMotorControlMode.CURRENT

    def autopilot_parameters_callback(self, new_parameters: String) -> None:
        """
        Update the autopilot parameter set at runtime.

        Parameters
        ----------
        new_parameters
            JSON-formatted string containing the parameter updates to apply.
        """
        new_parameters_json: dict = json.loads(new_parameters.data)

        for new_parameter_name, new_parameter_value in new_parameters_json.items():
            if new_parameter_name not in self.autopilot_parameters:
                print("WARNING: Attempted to set an autopilot parameter that the autopilot doesn't know")
                continue
            self.autopilot_parameters[new_parameter_name] = new_parameter_value

        if "autopilot_refresh_rate" in new_parameters_json:
            self.destroy_timer(self.autopilot_refresh_timer)
            self.autopilot_refresh_timer = self.create_timer(
                1 / self.autopilot_parameters["autopilot_refresh_rate"], self.update_ros_topics
            )

    def waypoints_list_callback(self, waypoint_list: WaypointList) -> None:
        """
        Receive and store a waypoint list for the mission planner.

        Parameters
        ----------
        waypoint_list
            ROS message containing a sequence of GPS waypoints for the boat to
            follow.
        """
        if len(waypoint_list.waypoints) == 0:
            self.differential_thrust_motorboat_autopilot.reset()
            return

        self.differential_thrust_motorboat_autopilot.reset()

        waypoint_navsatfixes: list[NavSatFix] = waypoint_list.waypoints
        waypoint_positions: list[Position] = []
        for waypoint in waypoint_navsatfixes:
            waypoint_positions.append(Position(longitude=waypoint.longitude, latitude=waypoint.latitude))

        self.differential_thrust_motorboat_autopilot.update_waypoints_list(waypoint_positions)

    def position_callback(self, position: NavSatFix) -> None:
        """
        Store the current GPS position for navigation and mission tracking.

        Parameters
        ----------
        position
            ROS NavSatFix message with the latest latitude and longitude values.
        """
        self.position = Position(longitude=position.longitude, latitude=position.latitude)
        self.last_gps_position_received_time = time.time()

    def velocity_callback(self, velocity: Twist) -> None:
        """
        Store the current planar velocity vector and speed.

        Parameters
        ----------
        velocity
            Twist message containing the latest linear velocity data.
        """
        self.velocity = np.array([velocity.linear.x, velocity.linear.y], dtype=np.float64)
        self.speed = np.linalg.norm(self.velocity)

    def heading_callback(self, heading: Float32) -> None:
        """
        Store the current boat heading.

        Parameters
        ----------
        heading
            Float32 message containing the current heading in degrees.
        """
        self.heading = heading.data
        self.last_heading_received_time = time.time()

    def emergency_stop_callback(self, should_emergency_stop: Bool) -> None:
        """
        Track whether an emergency stop is active.

        Parameters
        ----------
        should_emergency_stop
            Boolean indicating whether the perception or obstacle system has
            requested an emergency stop.
        """
        self.emergency_stop_active = should_emergency_stop.data

    def update_ros_topics(self) -> None:
        """
        Compute the control output for the current mode and publish it.

        This is the main loop of the node. It monitors sensor health, checks the
        active vehicle mode, and publishes the left/right motor commands needed
        for the differential-thrust boat to follow RC, hold heading, or navigate
        to waypoints.
        """
        # The node keeps a legacy rudder angle field for compatibility with the
        # rest of the telemetry and UI pipeline, even though the actual steering is
        # done by balancing the two motor commands rather than rotating a rudder.
        desired_rudder_angle = 0.0
        left_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)
        right_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)

        # Sensor health checks are still important for safe operation. If we lose
        # RC, GPS, or heading data, the boat should stop rather than continue
        # driving with stale target information.
        time_since_last_rc_data = time.time() - self.last_rc_data_received_time
        has_rc_data_disconnected = time_since_last_rc_data >= self.autopilot_parameters["rc_data_failsafe_time"]

        time_since_last_gps_data = time.time() - self.last_gps_position_received_time
        has_gps_disconnected = time_since_last_gps_data >= self.autopilot_parameters["gps_position_failsafe_time"]

        time_since_last_heading_data = time.time() - self.last_heading_received_time
        has_heading_disconnected = time_since_last_heading_data >= self.autopilot_parameters["heading_data_failsafe_time"]

        has_sensor_disconnected = has_rc_data_disconnected or has_gps_disconnected or has_heading_disconnected

        if not self.should_propeller_motor_be_powered or self.emergency_stop_active:
            left_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)
            right_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)

        elif has_sensor_disconnected:
            self.logger.info(f"has sensor disconnected: {has_sensor_disconnected}")
            self.logger.info(f"has remote controller disconnected: {has_rc_data_disconnected}")
            self.logger.info(f"has gps disconnected: {has_gps_disconnected}")
            self.logger.info(f"has heading disconnected: {has_heading_disconnected}")

            # Sensor timeout: stop both motors immediately to avoid driving with
            # stale heading or GPS information.
            left_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)
            right_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)

        elif self.motorboat_control_mode == MotorboatControlModes.EMERGENCY_STOP:
            # Emergency stop is a full stop state. The boat is commanded to a zero
            # thrust condition, though the legacy rudder field is still published
            # for compatibility with the rest of the system.
            left_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)
            right_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)

        elif (
            self.motorboat_control_mode == MotorboatControlModes.WAYPOINT_MISSION
            and self.differential_thrust_motorboat_autopilot.waypoints is not None
        ):
            # Autonomous waypoint mode: ask the differential-thrust controller for
            # the correct left/right thruster commands to steer toward the current
            # waypoint while maintaining forward progress.
            left_cmd, right_cmd, desired_rudder_angle = self.differential_thrust_motorboat_autopilot.run_waypoint_mission_step(
                self.position,
                self.heading,
            )
            left_control = VESCControlData(control_type_for_vesc="rpm", control_value=float(left_cmd))
            right_control = VESCControlData(control_type_for_vesc="rpm", control_value=float(right_cmd))

        elif self.motorboat_control_mode == MotorboatControlModes.HOLD_HEADING:
            # Hold-heading mode uses the same heading PID as the original boat,
            # but instead of moving a rudder, it converts that correction into a
            # left/right steering split for the propulsion motors.
            desired_vesc_control_type, desired_left, desired_right = (
                self.differential_thrust_motorboat_autopilot.run_hold_heading_step(
                    self.joystick_left_y,
                    self.heading,
                    self.heading_to_hold,
                    self.propeller_motor_control_mode,
                )
            )
            left_control = VESCControlData(control_type_for_vesc=desired_vesc_control_type, control_value=desired_left)
            right_control = VESCControlData(control_type_for_vesc=desired_vesc_control_type, control_value=desired_right)

        elif self.motorboat_control_mode == MotorboatControlModes.FULL_RC:
            # Full RC mode bypasses the autonomous logic and simply maps the
            # joystick input into two motor commands that create the desired turn
            # and forward/backward motion on the differential-thrust platform.
            desired_vesc_control_type, desired_left, desired_right = self.differential_thrust_motorboat_autopilot.run_rc_control(
                self.joystick_left_y,
                self.joystick_right_x,
                self.propeller_motor_control_mode,
            )
            left_control = VESCControlData(control_type_for_vesc=desired_vesc_control_type, control_value=desired_left)
            right_control = VESCControlData(control_type_for_vesc=desired_vesc_control_type, control_value=desired_right)

        else:
            left_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)
            right_control = VESCControlData(control_type_for_vesc="rpm", control_value=0.0)

        # Publish the actual motor commands to the left and right thrusters.
        self.left_propeller_motor_control_struct_publisher.publish(left_control)
        self.right_propeller_motor_control_struct_publisher.publish(right_control)

        # Keep legacy telemetry channels populated for compatibility with the rest
        # of the boat stack even though the physical steering is differential.
        if desired_rudder_angle is not None:
            self.desired_rudder_angle_publisher.publish(Float32(data=float(desired_rudder_angle)))

        if self.should_zero_encoder:
            self.zero_rudder_encoder_publisher.publish(Bool(data=self.should_zero_encoder))
            self.encoder_has_been_zeroed = True

        self.should_propeller_motor_be_powered_publisher.publish(Bool(data=self.should_propeller_motor_be_powered))

        self.current_waypoint_index_publisher.publish(Int32(data=self.differential_thrust_motorboat_autopilot.current_waypoint_index))
        control_mode = (
            MotorboatControlModes.EMERGENCY_STOP
            if self.emergency_stop_active
            else self.motorboat_control_mode
        )
        self.boat_control_mode_publisher.publish(String(data=control_mode.name))

        # Publish the desired heading to the rest of the stack. In hold-heading mode
        # this is the captured target heading; in waypoint mode it is the bearing to
        # the current target waypoint; otherwise it is zeroed.
        if self.motorboat_control_mode == MotorboatControlModes.HOLD_HEADING:
            self.desired_heading_publisher.publish(Float32(data=float(self.heading_to_hold)))
        elif (
            self.motorboat_control_mode == MotorboatControlModes.WAYPOINT_MISSION
            and self.differential_thrust_motorboat_autopilot.waypoints is not None
        ):
            current_waypoint_index = (
                self.differential_thrust_motorboat_autopilot.current_waypoint_index
            )
            current_waypoint = self.differential_thrust_motorboat_autopilot.waypoints[
                current_waypoint_index
            ]
            bearing_to_waypoint = get_bearing(self.position, current_waypoint)
            self.desired_heading_publisher.publish(Float32(data=float(bearing_to_waypoint)))
        else:
            self.desired_heading_publisher.publish(Float32(data=0.0))


def main() -> None:
    rclpy.init()
    differential_thrust_motorboat_autopilot_node = DifferentialThrustMotorboatAutopilotNode()
    rclpy.spin(differential_thrust_motorboat_autopilot_node)

    differential_thrust_motorboat_autopilot_node.destroy_node()
    rclpy.shutdown()
