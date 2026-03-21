import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from autoboat_msgs.msg import WaypointList


class ObstaclesNode(Node):

    def __init__(self):
        super().__init__("sailboat_obstacles")
        self.get_logger().info("working")

        self.obstacles_list_subscriber = self.create_subscription(NavSatFix, "/new_obstacle", self.update_obstacle_list, 10)
        self.obstacles_list_publisher = self.create_publisher(WaypointList, "/obstacles_list", 10)
        self.current_obstacles: list[tuple[float, float]] = []

        self.publisher_ = self.create_publisher(NavSatFix, 'new_obstacle', 10)

    def update_obstacle_list(self, msg: NavSatFix):
        self.current_obstacles.append((msg.latitude, msg.longitude))

        obstacles_nav_sat_fix_list = [
            NavSatFix(latitude=waypoint[0], longitude=waypoint[1]) for waypoint in self.current_obstacles
        ]
        self.obstacles_list_publisher.publish(WaypointList(waypoints = obstacles_nav_sat_fix_list))
        self.get_logger().info(f'Current waypoints: {self.current_obstacles}')

def main():
    rclpy.init()
    node = ObstaclesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()