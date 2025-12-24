import json
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, Int32, String

from autoboat_msgs.msg import RCData, VESCControlData, WaypointList

from .autopilot_library.discrete_pid import Discrete_PID
from .autopilot_library.motorboat_autopilot import MotorboatAutopilot
from .autopilot_library.utils import *


class MotorboatAutopilotNode(Node):
    """
    Handles communications between the autopilot (`autopilot_library/motorboat_autopilot.py`) and all of the other nodes/topics through ROS2.
    
    The autopilot takes in a bunch of sensor data and waypoints (list of gps positions that represent waypoints) and attempts to traverses through the waypoints by continuously publishing to the propeller motor control struct and rudder angle topics.

    The main function that you should pay attention to is `update_ros_topics`, since this is the function that is called periodically on a timer.

    NOTE: All units are in standard SI units and angles are generally measured in degrees unless otherwise specified.
    """

    def __init__(self) -> None:
        super().__init__("motorboat_autopilot")

        cur_folder_path = os.path.dirname(os.path.realpath(__file__))
        with open(cur_folder_path + "/config/motorboat_default_parameters.yaml", "r") as stream:
            self.parameters: dict = yaml.safe_load(stream)

        # this is temporarily using the sailboat autopilot object for now since we still need to implement the motorboat autopilot object
        self.motorboat_autopilot = MotorboatAutopilot(parameters=self.parameters, logger=self.get_logger())

        # Initialize ros2 subscriptions, publishers, and timers
        self.autopilot_refresh_timer = self.create_timer(
            1 / self.parameters["autopilot_refresh_rate"], self.update_ros_topics
        )

        self.autopilot_parameters_listener = self.create_subscription(
            String, "/autopilot_parameters", callback=self.autopilot_parameters_callback, qos_profile=10
        )
        self.waypoints_list_listener = self.create_subscription(
            WaypointList, "/waypoints_list", self.waypoints_list_callback, 10
        )

        self.position_listener = self.create_subscription(
            msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=qos_profile_sensor_data
        )
        self.velocity_listener = self.create_subscription(
            msg_type=Twist, topic="/velocity", callback=self.velocity_callback, qos_profile=qos_profile_sensor_data
        )
        self.heading_listener = self.create_subscription(
            msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=qos_profile_sensor_data
        )
        self.rc_data_listener = self.create_subscription(
            msg_type=RCData, topic="/rc_data", callback=self.rc_data_callback, qos_profile=qos_profile_sensor_data
        )

        self.current_waypoint_index_publisher = self.create_publisher(Int32, "/current_waypoint_index", 10)
        self.autopilot_mode_publisher = self.create_publisher(
            String, "/autopilot_mode", qos_profile=qos_profile_sensor_data
        )
        self.full_autonomy_maneuver_publisher = self.create_publisher(
            msg_type=String, topic="/full_autonomy_maneuver", qos_profile=qos_profile_sensor_data
        )
        self.desired_heading_publisher = self.create_publisher(Float32, "/desired_heading", qos_profile=10)

        self.should_propeller_motor_be_powered_publisher = self.create_publisher(
            Bool, "/should_propeller_motor_be_powered", qos_profile=10
        )
        self.propeller_motor_control_struct_publisher = self.create_publisher(
            msg_type=VESCControlData, topic="/propeller_motor_control_struct", qos_profile=qos_profile_sensor_data
        )
        self.desired_rudder_angle_publisher = self.create_publisher(
            msg_type=Float32, topic="/desired_rudder_angle", qos_profile=qos_profile_sensor_data
        )

        self.zero_rudder_encoder_publisher = self.create_publisher(
            msg_type=Bool, topic="/zero_rudder_encoder", qos_profile=10
        )

        self.heading_pid_controller = Discrete_PID(
            sample_period=(1 / self.parameters["autopilot_refresh_rate"]), Kp=1, Ki=0, Kd=0, n=1
        )

        self.max_rpm = 10000

        # default values
        self.position = Position(longitude=0.0, latitude=0.0)
        self.velocity = np.array([0.0, 0.0])
        self.speed = 0.0
        self.heading = 0.0
        self.rudder_angle = 0.0

        self.autopilot_mode = MotorboatAutopilotMode.Hold_Heading
        self.should_propeller_motor_be_powered = False
        self.should_zero_encoder = False
        self.encoder_has_been_zeroed = False
        self.heading_to_hold = 0.0

        self.last_rc_data_time = 0.0  # used to check whether we have disconnected from the remote controller
        self.joystick_left_x = 0.0
        self.joystick_left_y = 0.0
        self.joystick_right_x = 0.0
        self.joystick_right_y = 0.0

        self.button_a = 0
        self.toggle_b = 0
        self.toggle_c = 0
        self.button_d = 0
        self.toggle_e = 0
        self.toggle_f = 0

        self.propeller_motor_control_mode: MotorboatControls = MotorboatControls.RPM

    def rc_data_callback(self, rc_data_message: RCData) -> None:
        """
        This callback is called whenever there is new data about what is being pressed on the remote control.
        Whenever this callback is called, a couple of things happen: first, we check whether or not we are trying to zero
        the rudder and if we have, then we should handle that. Second, we need to keep track of if we are entering
        hold heading mode, and if we are, then we need to keep track of the current heading, since that heading is the one we will need to hold.
        Thirdly, we set the SailboatAutopilotMode based on a specific combination of toggles to determine whether we are in full rc mode,
        full autopilot mode, or one of the semi autonomous modes. Finally, we need to set the propeller control mode based on the toggles
        to determine whether we are controlling the propeller based on current output, duty cycle output, or propeller RPM.

        Args:
            rc_data_message (RCData): A struct that contains all of the data on what is pressed on the remote control
        """

        self.last_rc_data_time = time.time()

        # This means we have entered hold heading mode, so keep track of the current heading since this is the target heading
        if rc_data_message.toggle_f == 1 and self.toggle_f != 1:
            self.heading_to_hold = self.heading

        # Are we trying to zero the rudder?
        if self.button_d == False and rc_data_message.button_d == True:
            self.should_zero_encoder = True
            self.encoder_has_been_zeroed = False
        elif self.encoder_has_been_zeroed:
            self.should_zero_encoder = False

        self.joystick_left_x = rc_data_message.joystick_left_x
        self.joystick_left_y = (
            -1 * rc_data_message.joystick_left_y
        )  # this is negated because positive throttle ends up making the boat go in reverse
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
            self.autopilot_mode = MotorboatAutopilotMode.Disabled

        # full autonomy
        elif self.toggle_f == 2:
            self.autopilot_mode = MotorboatAutopilotMode.Waypoint_Mission

        # hold heading to the direction that we started this mode in
        elif self.toggle_f == 1:
            self.autopilot_mode = MotorboatAutopilotMode.Hold_Heading

        # remote controlled
        elif self.toggle_f == 0:
            self.autopilot_mode = MotorboatAutopilotMode.Full_RC
        # should not happen
        else:
            print("WARNING: INCORRECT COMBINATION OF RC SWITCHES USED")

        # Check the type of control that is selected
        if self.toggle_c == 0:
            self.propeller_motor_control_mode = MotorboatControls.RPM
        elif self.toggle_c == 1:
            self.propeller_motor_control_mode = MotorboatControls.DUTY_CYCLE
        elif self.toggle_c == 2:
            self.propeller_motor_control_mode = MotorboatControls.CURRENT

    def autopilot_parameters_callback(self, new_parameters: String) -> None:
        """
        Receives a serialized json (as a string) of parameters and sets them as constants.
        Any constant can be set as long as they are in the json
        """
        new_parameters_json: dict = json.loads(new_parameters.data)
        for new_parameter_name, new_parameter_value in new_parameters_json.items():
            if new_parameter_name not in self.parameters.keys():
                print("WARNING: Attempted to set an autopilot parameter that the autopilot doesn't know")
                print("If you would like to make a new autopilot parameter, please edit default_parameters.yaml")
                continue

            self.parameters[new_parameter_name] = new_parameter_value

        # special cases to handle since they do not update automatically
        if "autopilot_refresh_rate" in new_parameters_json:
            self.destroy_timer(self.autopilot_refresh_timer)
            self.autopilot_refresh_timer = self.create_timer(
                1 / self.parameters["autopilot_refresh_rate"], self.update_ros_topics
            )

    def waypoints_list_callback(self, waypoint_list: WaypointList) -> None:
        """
        Convert the list of ROS2 Nav Sat Fix objects to a list of Position objects, which are a custom datatype that has some useful helper methods.
        The Position object should be simpler to do calculations with
        """
        if len(waypoint_list.waypoints) == 0:
            return

        self.motorboat_autopilot.reset()

        waypoint_navsatfixes: list[NavSatFix] = waypoint_list.waypoints
        waypoint_positions: list[Position] = []

        for navsatfix in waypoint_navsatfixes:
            waypoint_positions.append(Position(navsatfix.longitude, navsatfix.latitude))

        self.motorboat_autopilot.update_waypoints_list(waypoint_positions)

    def position_callback(self, position: NavSatFix):
        self.position = Position(longitude=position.longitude, latitude=position.latitude)

    def velocity_callback(self, velocity: Twist):
        self.velocity = np.array([velocity.linear.x, velocity.linear.y])
        self.speed = np.sqrt(velocity.linear.x**2 + velocity.linear.y**2)

    def heading_callback(self, heading: Float32):
        self.heading = heading.data

    def get_optimal_rudder_angle(self, heading, desired_heading):
        """
        Args:
            heading (float): the current heading of the boat measured counter-clockwise from true east
            desired_heading (float): the current desired heading of the boat measured counter-clockwise from true east

        Returns:
            float: the angle we should turn the rudder in order to turn from our current heading to the desired heading
        """
        error = get_distance_between_angles(desired_heading, heading)

        # ensure that the gains are updated properly and if we get a new gain from the telemetry server, it gets properly updated
        self.heading_pid_controller.set_gains(
            Kp=self.parameters["heading_p_gain"],
            Ki=self.parameters["heading_i_gain"],
            Kd=self.parameters["heading_d_gain"],
            n=self.parameters["heading_n_gain"],
            sample_period=self.parameters["autopilot_refresh_rate"],
        )

        rudder_angle = self.heading_pid_controller(error)
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
        rudder_angle = (
            ((joystick_right_x - -100) * (max_rudder_angle - min_rudder_angle)) / (100 - -100)
        ) + min_rudder_angle

        return sail_angle, rudder_angle

    def step(self) -> float:
        """
        TODO Document what this function does maybe?
        """

        if (
            self.autopilot_mode == MotorboatAutopilotMode.Waypoint_Mission
            and self.motorboat_autopilot.waypoints != None
        ):
            return 0.0  # this is unimplemented for now
            # _, rudder_angle = self.motorboat_autopilot.run_waypoint_mission_step(self.position, self.velocity, self.heading, self.apparent_wind_angle)

        elif self.autopilot_mode == MotorboatAutopilotMode.Hold_Heading:
            rudder_angle = self.get_optimal_rudder_angle(self.heading, self.heading_to_hold)

        elif self.autopilot_mode == MotorboatAutopilotMode.Full_RC:
            _, rudder_angle = self.run_rc_control(self.joystick_left_y, self.joystick_right_x)

        else:
            return None

        return rudder_angle

    def update_ros_topics(self) -> None:
        """
        This is the main function that is called constantely by the timer.
        Updates the propeller control struct and rudder angle topics based on the output of the autopilot controller
        """

        desired_rudder_angle = self.step()

        # self.current_waypoint_index_publisher.publish(Int32(data=self.motorboat_autopilot.current_waypoint_index))

        # self.autopilot_mode_publisher.publish(String(data=self.autopilot_mode.name))
        # if self.autopilot_mode == MotorboatAutopilotMode.Waypoint_Mission:
        #     self.full_autonomy_maneuver_publisher.publish(String(data=self.motorboat_autopilot.current_state.name))
        # else:
        #     self.full_autonomy_maneuver_publisher.publish(String(data="N/A"))

        if self.autopilot_mode == MotorboatAutopilotMode.Hold_Heading:
            self.desired_heading_publisher.publish(Float32(data=float(self.heading_to_hold)))

        # elif self.autopilot_mode == MotorboatAutopilotMode.Waypoint_Mission and self.motorboat_autopilot.waypoints != None:
        #     current_waypoint = self.motorboat_autopilot.waypoints[self.motorboat_autopilot.current_waypoint_index]
        #     bearing_to_waypoint = get_bearing(self.position, current_waypoint) #TODO make it so that this is the actual heading the autopilot is trying to follow (this is different when tacking)
        #     self.desired_heading_publisher.publish(Float32(data=float(bearing_to_waypoint)))

        # else:
        #     self.desired_heading_publisher.publish(Float32(data=0.))

        if desired_rudder_angle != None:
            self.desired_rudder_angle_publisher.publish(Float32(data=float(desired_rudder_angle)))

        # Manually check whether the RC has disconnected if we have not received data from the remote control for 3 second
        has_rc_disconnected = False
        if time.time() - self.last_rc_data_time >= 3:
            has_rc_disconnected = True

            self.propeller_motor_control_struct_publisher.publish(
                VESCControlData(
                    control_type_for_vesc="rpm",
                    desired_vesc_current=0.0,
                    desired_vesc_rpm=0.0,
                    desired_vesc_duty_cycle=0.0,
                )
            )

        # Now it is time to apply the RC control, check the control type, and then tell the VESC node to turn
        if self.autopilot_mode == MotorboatAutopilotMode.Full_RC and not has_rc_disconnected:
            if self.propeller_motor_control_mode == MotorboatControls.RPM:
                rpm_value = 100.0 * self.joystick_left_y  # min -1e5 max 1e5

                self.propeller_motor_control_struct_publisher.publish(
                    VESCControlData(
                        control_type_for_vesc="rpm",
                        desired_vesc_current=0.0,
                        desired_vesc_rpm=rpm_value,
                        desired_vesc_duty_cycle=0.0,
                    )
                )

            elif self.propeller_motor_control_mode == MotorboatControls.DUTY_CYCLE:
                duty_cycle_value = self.joystick_left_y  # min 0 max 100

                self.propeller_motor_control_struct_publisher.publish(
                    VESCControlData(
                        control_type_for_vesc="duty_value",
                        desired_vesc_current=duty_cycle_value,
                        desired_vesc_rpm=0.0,
                        desired_vesc_duty_cycle=0.0,
                    )
                )

            elif self.propeller_motor_control_mode == MotorboatControls.CURRENT:
                current_value = self.joystick_left_y
                self.propeller_motor_control_struct_publisher.publish(
                    VESCControlData(
                        control_type_for_vesc="current",
                        desired_vesc_current=current_value,
                        desired_vesc_rpm=0.0,
                        desired_vesc_duty_cycle=0.0,
                    )
                )

        self.should_propeller_motor_be_powered_publisher.publish(Bool(data=self.should_propeller_motor_be_powered))

        if self.should_zero_encoder:
            self.zero_rudder_encoder_publisher.publish(Bool(data=self.should_zero_encoder))
            self.encoder_has_been_zeroed = True


def main() -> None:
    rclpy.init()
    motorboat_autopilot_node = MotorboatAutopilotNode()
    rclpy.spin(motorboat_autopilot_node)

    motorboat_autopilot_node.destroy_node()
    rclpy.shutdown()
