#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber") # Node name
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10) # Callback argument to process the msg wehn received
    
    def pose_callback(self, msg: Pose):
        #self.get_logger().info(str(msg))
        self.get_logger().info("(" + str(msg.x) + "," + str(msg.y) + ")")


def main(args=None):
    rclpy.init(args=args) # initialize ROS2 communications
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown() # Last line