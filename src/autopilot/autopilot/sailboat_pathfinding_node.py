import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from autoboat_msgs.msg import WaypointList
from rclpy.qos import qos_profile_sensor_data

class PathfindingNode(Node):

    def __init__(self):
        super().__init__("sailboat_pathfinding")
        self.get_logger().info("working")
        # create publisher stuff here (later)

        # subscribing to position
        self.gps_subscriber=self.create_subscription(NavSatFix, "/position", self.gps_call, qos_profile_sensor_data)
        self.waypoint_subscriber=self.create_subscription(WaypointList, "/waypoints_list", self.waypointcall, qos_profile_sensor_data)

    def gps_call(self,msg: NavSatFix): # argument in function is a shorthand for calling msg as object of NavSatFix
        # self.get_logger().info("latitude: " + str(msg.latitude) + " longitude: " + str(msg.longitude))
        pass
        
    def waypointcall(self,msg: WaypointList): # function calling for waypoints once sent
        self.get_logger().info(str(msg))

def main():
    rclpy.init()
    node = PathfindingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()