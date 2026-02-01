import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from autoboat_msgs.msg import WaypointList
from rclpy.qos import qos_profile_sensor_data
from .autopilot_library.utils import *
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
        self.res=10000
        self.origin = [0,0]
        self.destinations=[]
        self.wayindex=0

    def gps_call(self,msg: NavSatFix): # argument in function is a shorthand for calling msg as object of NavSatFix
        # self.get_logger().info("latitude: " + str(msg.latitude) + " longitude: " + str(msg.longitude))
        self.origin = [msg.longitude*self.res,msg.latitude*self.res]
        
    def waypoint_call(self,msg: WaypointList): # function calling for waypoints once sent
        self.wayindex=0
        self.destinations = [[msg.waypoints[i].longitude*self.res,msg.waypoints[i].latitude*self.res] for i in range(len(msg.waypoints))]
        self.make_matrix(self.origin,self.destinations[0])
    
    def make_matrix(self, src, des):
        self.get_logger().info(str(src))
        self.get_logger().info(str(des))
        lims=int(math.ceil(math.sqrt((des[0]-src[0])**2+(des[1]-src[1])**2)))+1
        xdist=int(math.floor(des[0]-src[0]))+lims-1
        ydist=int(math.floor(des[1]-src[1]))+lims-1

        matrix=[[0 for _ in range(lims*2)] for _ in range(lims*2)]
        self.get_logger().info(str(lims-1))
        self.get_logger().info(str(xdist) + " " + str(ydist))
        matrix[lims-1][lims-1]=1
        matrix[xdist][ydist]=2
        self.get_logger().info(str(matrix))

    def runpath(self):
        if self.destinations:
            dist=math.sqrt((self.destinations[self.wayindex][0]-self.origin[0])**2+(self.destinations[self.wayindex][1]-self.origin[1])**2)
            self.get_logger().info("\non waypoint " + str(self.wayindex+1) + "\nlongitude: " + str(self.destinations[self.wayindex][0]) + "\nlatitude: " + str(self.destinations[self.wayindex][1]) + "\ndistance: " + str(dist))
            if dist < 0.5 and self.wayindex < len(self.destinations)-1:
                self.wayindex+= 1
                self.make_matrix(self.origin, self.destinations[self.wayindex])
    

    # checklist
    # -------------------------------------------------------------------------------------------------
    # publish intermediate waypoints list ( check telemetry 467)

    # use position methods to convert degrees to local and otherwise ( check bottom of position node )
    # use 1st waypoint as reference
                
def main():
    rclpy.init()
    node = PathfindingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()