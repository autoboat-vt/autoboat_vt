import rclpy
from rclpy.node import Node

class PathNode(Node):

    def __init__(self):
        super().__init__("sailboat_pathfinding")
        self.get_logger().info("working")
        # create publisher stuff here
        

def main():
    rclpy.init()
    node = PathNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()