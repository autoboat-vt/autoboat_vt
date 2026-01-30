import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from autoboat_msgs.msg import WaypointList
from rclpy.qos import qos_profile_sensor_data
import math

class PathfindingNode(Node):

    def __init__(self):
        super().__init__("sailboat_pathfinding")
        self.get_logger().info("working")
        # create publisher stuff here (later)

        # subscribing to position
        self.gps_subscriber=self.create_subscription(NavSatFix, "/position", self.gps_call, qos_profile_sensor_data)
        self.waypoint_subscriber=self.create_subscription(WaypointList, "/waypoints_list", self.waypoint_call, qos_profile_sensor_data)
        self.create_timer(1.0,self.runpath)

        # creating origin for matrix
        self.degconv=10000
        self.origin = [0,0]
        self.destinations=[]

    def gps_call(self,msg: NavSatFix): # argument in function is a shorthand for calling msg as object of NavSatFix
        # self.get_logger().info("latitude: " + str(msg.latitude) + " longitude: " + str(msg.longitude))
        self.origin = [msg.longitude*self.degconv,msg.latitude*self.degconv]
        
    def waypoint_call(self,msg: WaypointList): # function calling for waypoints once sent
        self.get_logger().info(str(len(msg.waypoints)))
        self.destinations = [[msg.waypoints[i].longitude*self.degconv,msg.waypoints[i].latitude*self.degconv] for i in range(len(msg.waypoints))]

    def checkhit(self, index):
        dist=math.sqrt((self.destinations[index][0]-self.origin[0])**2+(self.destinations[index][1]-self.origin[1])**2)

        if dist > 5.0:
            return 1
        return 0

    def runpath(self):
        if self.destinations:
            for i in range(len(self.destinations)):
                while self.checkhit(i):
                    self.get_logger().info("\non " + str(i+1) + " waypoint" + "\nlongitude: " + str(self.destinations[i][0]) + "\nlatitude: " + str(self.destinations[i][1]))
        
        self.get_logger().info("done")

def main():
    rclpy.init()
    node = PathfindingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()