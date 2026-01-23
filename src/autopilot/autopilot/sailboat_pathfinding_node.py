import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
from .autopilot_library.utils import *

class PathfindingNode(Node):

    def __init__(self):
        super().__init__("sailboat_pathfinding")
        self.get_logger().info("working")
        # create publisher stuff here (later)

        # subscribing to position
        self.gps_subscriber=self.create_subscription(NavSatFix, "/position", self.gps_call, qos_profile_sensor_data)

    def gps_call(self,msg: NavSatFix): # argument in function is a shorthand for calling msg as object of NavSatFix
        self.get_logger().info("latitude: " + str(msg.latitude) + " longitude: " + str(msg.longitude))
        
def main():
    rclpy.init()
    node = PathfindingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()