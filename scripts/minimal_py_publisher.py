#! /usr/bin/env python3

"""
Description:
    This ROS 2 node periodically publishes "Hello World" messages on a topic.
    It demonstrates basic ROS concepts such as node creation, publishing, and
    timer usage.
-------
Publishing Topics:
    The channel containing the "Hello World" messages
    /topic - std_msgs/String
-------
Subscription Topics:
    None    
-------
Author: Addison Sears-Collins
Date: January 31, 2024
"""

import rclpy # Import the ROS 2 client library for Python
from rclpy.node import Node # Import the Node class for creating ROS 2 nodes

from std_msgs.msg import String # Import the String message type for publishing
from nav_msgs.msg import Odometry


class OdometryPublisher(Node):
    """Create MinimalPublisher node.

    """
     def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odometry', 10)

    def publish_odometry(self, x, y, z, quat_x, quat_y, quat_z, quat_w):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quat_x
        msg.pose.pose.orientation.y = quat_y
        msg.pose.pose.orientation.z = quat_z
        msg.pose.pose.orientation.w = quat_w
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
