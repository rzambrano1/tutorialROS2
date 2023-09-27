#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node): # For programming purposes

    def __init__(self):
        super().__init__("first_node") # Node name
        # self.get_logger().info("Hello from ROS2")# To make the node do something ## Removed when timer_callback was created
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback) # The callback is enable by spin

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args) # initialize ROS2 communications
    node = MyNode()
    rclpy.spin(node) # With this line the node is kept alive and will continue to run up until it is killed
    rclpy.shutdown() # Last line

if __name__=='__main__':
    main()


# We need to install the node in order to use it with ROS commands - go to setup.py add test_node in entry_points => console_scripts with quotes
# It is called an executable table 
# Any change in this script needs to be followed by 
# $ colcon build --symlink-install 
# $ source ~/.bashrc
# After running $--symlink-install no need to build again 