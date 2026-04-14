import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64

from autoboat_msgs.msg import VESCControlData

from .utils import cartesian_vector_to_polar, euler_from_quaternion


class AutopilotTransformNode(Node):

    def __init__(self) -> None:
        super().__init__("autopilot_transform_node")

        self.autopilot_transform_refresh_timer = self.create_timer(0.1, self.update_ros_topics)

        self.velocity_publisher =  self.create_publisher(Twist, "/velocity", 10)
        self.heading_publisher = self.create_publisher(Float32, "/heading", 10)
        self.desired_rudder_angle_publisher = self.create_publisher(Float64, "/motorboat_simulation/desired_rudder_angle", 10)
        self.propeller_rpm_publisher = self.create_publisher(Float64, "/motorboat_simulation/desired_propeller_rpm", 10)
        self.current_rudder_angle_publisher = self.create_publisher(Float32, "/current_rudder_angle", 10)
        
        self.create_subscription(JointState, "/world/motorboat_model/model/my_ship/joint_state", self.joint_state_callback, 10)
        self.create_subscription(Odometry, "/motorboat_simulation/odometry", self.odometry_callback, 10)
        self.create_subscription(Float32, "/desired_rudder_angle", self.desired_rudder_callback, 10)
        self.create_subscription(
            VESCControlData, "/propeller_motor_control_struct", self.vesc_control_data_callback, qos_profile_sensor_data
        )

        self.velocity = Twist()
        self.speed= 0.0
        self.heading:float  = 0.0
        self.odometry = Odometry()
        self.desired_rudder_angle: float = 0.0
        self.vesc_control_data: VESCControlData = VESCControlData()
        self.joint_state: JointState = JointState()
    

    def odometry_callback(self, odometry: Odometry) -> None:
        """A callback function to get the current odometry of the boat."""
        self.odometry = odometry

    def desired_rudder_callback(self, desired_rudder_angle: Float32) -> None:
        """A callback function to get the desired rudder angle of the boat."""
        self.desired_rudder_angle = np.deg2rad(float(desired_rudder_angle.data))
    
    def vesc_control_data_callback(self, vesc_control_data: VESCControlData) -> None:
        """A callback function to get the current propeller motor control the autopilot is attempting to output."""
        self.vesc_control_data = vesc_control_data
    
    def joint_state_callback(self, joint_state: JointState) -> None:
        """A callback function to get the current joint states of the boat."""
        self.joint_state = joint_state

    def update_ros_topics(self) -> None:
        """
        A periodically called function that publishes data to the autopilot.
        This published data is in a format that the autopilot can natively understand and
        lets the autopilot node communicate with the gazebo simulation.
        """

        twist = Twist()
        twist.linear = self.odometry.twist.twist.linear
        twist.angular = self.odometry.twist.twist.angular
        velocity = twist
        self.velocity = velocity

        magnitude, direction = cartesian_vector_to_polar(self.velocity.linear.x,self.velocity.linear.y)
        
        direction += self.heading

        self.velocity.linear.x = float(magnitude* np.cos(np.deg2rad(float(direction))))
        self.velocity.linear.y = float(magnitude* np.sin(np.deg2rad(float(direction))))

        self.velocity_publisher.publish(self.velocity)


        pose = self.odometry.pose.pose.orientation
        x = pose.x
        y = pose.y
        z = pose.z
        w = pose.w
        
        _, _, yaw = np.rad2deg(euler_from_quaternion(x,y,z,w))
        # yaw = np.arctan2(2*(w*z + x*y) , 1 - 2*(pow(x,2) + pow(y, 2))) * 180 / np.pi % 360

        current_rudder_angle = 0.0 if len(self.joint_state.position) == 0 else np.rad2deg(self.joint_state.position[0])

        
        self.heading_publisher.publish(Float32(data=yaw))
        self.desired_rudder_angle_publisher.publish(Float64(data=self.desired_rudder_angle))
        self.propeller_rpm_publisher.publish(Float64(data=self.vesc_control_data.desired_vesc_rpm))
        self.current_rudder_angle_publisher.publish(Float32(data=current_rudder_angle))



def main() -> None:
    rclpy.init()
    autopilot_transform_node = AutopilotTransformNode()
    rclpy.spin(autopilot_transform_node)
    autopilot_transform_node.destroy_node()
    rclpy.shutdown()
