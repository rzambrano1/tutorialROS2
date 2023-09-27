#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # Include package in package.xml

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle") # Node name
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) # To find the msg_type if using an existing topic use
                                                    # $ ros2 topic list
                                                    # $ ros2 topic info /topic_name and see type in the resulting list | then import the msg type
                                                    # 10 is the queue type
                                                    # Need to create a callback to be able to send some data
        self.timer_ = self.create_timer(0.5, self.send_velocity_command) # Calls the function every 0.5 seconds
        self.get_logger().info("Draw circle node has been started") # This logger in the __init__ function allows you to see the node has started and is running

    def send_velocity_command(self):
        msg = Twist() # Creates the message | Need to use $ ros2 interface show geometry_msgs/msg/Twist to see what we need to send in the message
        msg.linear.x = 2.0 # Fills the data of the message
        msg.angular.z = 1.0 # Fills the data of the message
        self.cmd_vel_pub_.publish(msg) # Publish the message


def main(args=None):
    rclpy.init(args=args) # initialize ROS2 communications
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown() # Last line

# if __name__=='__main__':
#     main()