


import rclpy
from rclpy.node import Node
from simulation.utils import cartesian_vector_to_polar
from std_msgs.msg import Float32,Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
# from autopilot import Position 
import json
import yaml 
import os
import time

import  numpy as np
class AutopilotTransformNode(Node):

    def __init__(self):
        super().__init__("autopilot_transform_node")
        

        cur_folder_path = os.path.dirname(os.path.realpath(__file__))
        with open(cur_folder_path + "/config/motorboat_default_parameters.yaml", "r") as stream:
            self.autopilot_parameters: dict = yaml.safe_load(stream)


        self.autopilot_transform_refresh_timer = self.create_timer(1 / self.autopilot_parameters["autopilot_refresh_rate"], self.update_ros_topics)

        self.velocity_publisher =  self.create_publisher(Twist, "/velocity",qos_profile=10) 
        self.odometry_listener = self.create_subscription(msg_type=Odometry, topic="/odometry", callback=self.odometry_callback, qos_profile=10)
        self.heading_publisher = self.create_publisher(Float32, "heading",qos_profile=10)
        self.rudder_publisher = self.create_publisher(msg_type=Float64, topic="/rudder", qos_profile=10)
        self.rudder_listener = self.create_subscription(msg_type=Float32,topic="/desired_rudder_angle", callback=self.rudder_callback, qos_profile=10)

        self.velocity = Twist()
        self.speed= 0.0
        self.heading = 0.0
        self.odometry = Odometry()
        self.rudder = 0.0
    

    def odometry_callback(self, odometry: Odometry):
        self.odometry = odometry

    def rudder_callback(self, rudder: Float32):
        self.rudder = float(rudder.data)
        self.rudder = np.deg2rad(self.rudder)
    

    def update_ros_topics(self):


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
        
        yaw = np.arctan2(2*(w*z + x*y) , 1 - 2*(pow(x,2) + pow(y, 2))) * 180 / np.pi % 360

        self.heading_publisher.publish(Float32(data=yaw))

        self.rudder_publisher.publish(Float64(data=self.rudder))

def main():
    rclpy.init()
    autopilot_transform_node = AutopilotTransformNode()
    rclpy.spin(autopilot_transform_node)
    autopilot_transform_node.destroy_node()
    rclpy.shutdown()



