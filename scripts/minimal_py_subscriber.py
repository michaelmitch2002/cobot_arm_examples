#! /usr/bin/env python3

"""
Description:
    This ROS 2 node subscribes to "Hello World" messages on a topic.
    It demonstrates basic ROS concepts such as node creation and subscribing.
-------
Publishing Topics:
    None
-------
Subscription Topics:
    The channel containing the "Hello World" messages
    /topic - std_msgs/String   
-------
Author: Addison Sears-Collins
Date: February 15, 2024
"""

import rclpy # Import the ROS 2 client library for Python
from rclpy.node import Node # Import the Node class for creating ROS 2 nodes

from std_msgs.msg import String # Import the String message type


class MinimalSubscriber(Node):
    """Create Minimal Subscriber node.

    """
    def __init__(self):
        """ Create a custom node class for subscribing

        """

        # Initialize the node with a name
        super().__init__('minimal_subscriber')

        # Creates a subscriber
        self.subscriber_1 = self.create_subscription(
            String, 
            '/topic', 
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        """Call this function every time a new message is published on
            the topic.

        """
        # Log a message indicating that the message has been published
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    """Main function to start the ROS 2 node.

    Args:
        args (List, optional): Command-line arguments. Defaults to None.
    """

    # Initialize ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    # Keep the node running and processing events.
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

    # Shutdown ROS 2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    # Execute the main function if the script is run directly
    main()