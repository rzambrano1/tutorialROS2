#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose # We included this when including the Twist message
from geometry_msgs.msg import Twist # Include package in package.xml
from turtlesim.srv import SetPen # We know this because we ran 
                                 # $ ros2 service list
                                 # $ ros2 service type /turtle1/set_pen
from functools import partial

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller") # Node name
        self.previous_x_ = 0
        self.cmd_vel_pub_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10) # Callback argument to process the msg wehn received

        self.get_logger().info("Turtle controller has been started") # This logger in the __init__ function allows you to see the node has started and is running

    def pose_callback(self, pose: Pose):
        cmd = Twist() # The callback receives a pose and then we send a command
        
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_pub_.publish(cmd)

        if pose.x > 5.5 and self.previous_x_ <= 5.5: # 'and self.previous_x_ <= 5.5' is added to avoid calling the service at the same rate they publish cmd [60 Hz]
            self.previous_x_ = pose.x
            self.get_logger().info("Set color to red")
            self.call_set_pen_service(255,0,0,3,0)
        elif pose.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set color to green")
            self.call_set_pen_service(0,255,0,3,0)

    def call_set_pen_service(self,r,g,b,width,off): # We know these arguments because we ran $ ros2 interface show turtlesim/srv/SetPen
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0): # We want to call the service when it is available otherwise we will get an error
            self.get_logger().warn("Waiting for the service...")
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request) # If one blocks the thread that is calling the service one might get errors (i.e. the response is never received). 
                                            # Because of that we use call_async
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future): # This is the callback for when the service replies with the response
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args) # initialize ROS2 communications
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown() # Last line