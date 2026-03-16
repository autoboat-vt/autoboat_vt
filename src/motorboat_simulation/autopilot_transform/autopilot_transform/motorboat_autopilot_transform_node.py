import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Float64

from autoboat_msgs.msg import VESCControlData

from .utils import cartesian_vector_to_polar, euler_from_quaternion


class AutopilotTransformNode(Node):

    def __init__(self) -> None:
        super().__init__("autopilot_transform_node")

        self.autopilot_transform_refresh_timer = self.create_timer(0.1, self.update_ros_topics)

        self.velocity_publisher =  self.create_publisher(Twist, "/velocity", 10)
        self.heading_publisher = self.create_publisher(Float32, "/heading", 10)
        self.rudder_angle_publisher = self.create_publisher(Float64, "/motorboat_simulation/desired_rudder_angle", 10)
        self.propeller_rpm_publisher = self.create_publisher(Float64, "/motorboat_simulation/desired_propeller_rpm", 10)
        
        self.create_subscription(Odometry, "/motorboat_simulation/odometry", self.odometry_callback, 10)
        self.create_subscription(Float32, "/desired_rudder_angle", self.rudder_callback, 10)
        self.create_subscription(
            VESCControlData, "/propeller_motor_control_struct", self.vesc_control_data_callback, qos_profile_sensor_data
        )

        self.velocity = Twist()
        self.speed= 0.0
        self.heading = 0.0
        self.odometry = Odometry()
        self.rudder_angle = 0.0
        self.vesc_control_data = VESCControlData()
    

    def odometry_callback(self, odometry: Odometry) -> None:
        """A callback function to get the current odometry of the boat."""
        self.odometry = odometry

    def rudder_callback(self, rudder_angle: Float32) -> None:
        """A callback function to get the current rudder angle of the boat."""
        self.rudder_angle = np.deg2rad(float(rudder_angle.data))
    
    def vesc_control_data_callback(self, vesc_control_data: VESCControlData) -> None:
        """A callback function to get the current propeller motor control the autopilot is attempting to output."""
        self.vesc_control_data = vesc_control_data
        
        

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

        self.heading_publisher.publish(Float32(data=yaw))
        self.rudder_angle_publisher.publish(Float64(data=self.rudder_angle))
        self.propeller_rpm_publisher.publish(Float64(data=self.vesc_control_data.desired_vesc_rpm))



def main() -> None:
    rclpy.init()
    autopilot_transform_node = AutopilotTransformNode()
    rclpy.spin(autopilot_transform_node)
    autopilot_transform_node.destroy_node()
    rclpy.shutdown()
