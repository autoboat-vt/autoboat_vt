import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from autoboat_msgs.msg import WaypointList

from .autopilot_library.utils import *
from .autopilot_library.utils.astar import Astar


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
        # self.res=10000 using the polar to cartesian coordinates, not using this

        # initializing variables needed across functions
        self.boat = []
        self.boatGPS=[]
        self.destinations=[]
        self.wayindex=0
        self.reference=[0,0]

    def gps_call(self,msg: NavSatFix): # argument in function is a shorthand for calling msg as object of NavSatFix
        # self.get_logger().info("latitude: " + str(msg.latitude) + " longitude: " + str(msg.longitude))
        self.boatGPS=[msg.longitude,msg.latitude]

        
    def waypoint_call(self,msg: WaypointList): # function calling for waypoints once sent
        self.wayindex=0     # resetting the index counter

        # creating list of position objects
        posList=[]
        for i in range(len(msg.waypoints)):
                posList.append(Position(msg.waypoints[i].longitude,msg.waypoints[i].latitude))

        # using position get local to convert gps into local coordinates for destinations list

        # reference 1st waypoint
        self.destinations.append([0,0])
        self.reference=[posList[0].longitude,posList[0].latitude]

        # local for rest of the waypoints
        for i in range(1,len(msg.waypoints)):
            [lon,lat]=posList[i].get_local_coordinates(self.reference)
            self.destinations.append([lon,lat])
    
    def make_matrix(self, src, des):

        # outputting the boat and destination waypoint
        self.get_logger().info(str(src))
        self.get_logger().info(str(des))

        # outputtin distance
        lims=int(math.ceil(math.sqrt((des[0]-src[0])**2+(des[1]-src[1])**2)))+1

        # checking distances
        xdist=int(math.floor(des[0]-src[0]))+lims-1
        ydist=int(math.floor(des[1]-src[1]))+lims-1

        # creating matrix
        matrix=[[1 for _ in range(lims*2)] for _ in range(lims*2)]
        self.get_logger().info(str(lims-1))
        self.get_logger().info(str(xdist) + " " + str(ydist))
        
        # matrix[lims-1][lims-1]=2             this is the boat position
        # matrix[xdist][ydist]=3               this is the next waypoint position
        
        self.get_logger().info(str(matrix))

        # solving the matrix using Astar algo
        sol=Astar(matrix)
        self.get_logger().info(str(sol.astar([lims-1,lims-1],[xdist,ydist])))
    

    def runpath(self):

        # creating boat position object for local coordinate change
        boat = Position(self.boatGPS[0],self.boatGPS[1])
        self.boat = boat.get_local_coordinates(self.reference)

        # when destinations list is not empty, run the path
        if self.destinations:
            if self.wayindex==0:
                self.make_matrix(self.boat, self.destinations[0])
                self.wayindex += 1

            dist=math.sqrt((self.destinations[self.wayindex-1][0]-self.boat[0])**2+(self.destinations[self.wayindex-1][1]-self.boat[1])**2)
            self.get_logger().info("\non waypoint " + str(self.wayindex) + "\nlongitude: " + str(self.destinations[self.wayindex-1][0]) + "\nlatitude: " + str(self.destinations[self.wayindex-1][1]) + "\ndistance: " + str(dist))
            if dist < 10 and self.wayindex < len(self.destinations):
                self.make_matrix(self.boat, self.destinations[self.wayindex])
                self.wayindex+= 1
            
# ---------------------------------------------------------------- start of the astar algorithm-------------------------------------------------------------
    

    # checklist
    # -------------------------------------------------------------------------------------------------
    # run the astar algorithm with the matrix generated to make intermediate waypoints
    # publish intermediate waypoints list ( check telemetry 467)

    # add obstacles into the matrix (when the obstacles ROS topic is done)
    

                
def main():
    rclpy.init()
    node = PathfindingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()