#! /usr/bin/env python3

"""
Description:
    This ROS 2 node demonstrates how to declare parameters.
-------
Publishing Topics:
    None
-------
Subscription Topics:
    None    
-------
Author: Addison Sears-Collins
Date: March 4, 2024
"""

import rclpy # Import the ROS 2 client library for Python
from rclpy.node import Node # Import the Node class for creating ROS 2 nodes
from rcl_interfaces.msg import ParameterDescriptor # Enables the description of parameters
from rcl_interfaces.msg import SetParametersResult # Handles responses to parameter change requests
from rclpy.parameter import Parameter # Handles parameters within a node

class MinimalParameter(Node):
    """Create MinimalParameter node.

    """
    def __init__(self):
        """ Create a custom node class for declaring parameters.

        """

        # Initialize the node with a name
        super().__init__('minimal_parameter_node')

        # Describe parameters
        velocity_limit_descriptor = ParameterDescriptor(
            description='Maximum allowed angular velocity in radians per second (ignored for fixed joints)')        
        robot_name_descriptor = ParameterDescriptor(description='The name of the robot')

        # Declare parameters
        self.declare_parameter('velocity_limit', 2.792527, velocity_limit_descriptor)
        self.declare_parameter('robot_name', 'Automatic Addison Bot', robot_name_descriptor)

        # Register a callback function that will be called whenever there is an attempt to
        # change one or more parameters of the node.
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    def parameter_change_callback(self, params):
        """Gets called whenever there is an attempt to change one or more parameters.

        Args:
            params (List[Parameter]): A list of Parameter objects representing the parameters that are 
                being attempted to change.
        
        Returns:
            SetParametersResult: Object indicating whether the change was successful.
        """
        result = SetParametersResult()

        # Iterate over each parameter in this node
        for param in params:
            # Check the parameter's name and type
            if param.name == "velocity_limit" and param.type_ == Parameter.Type.DOUBLE:
                # This parameter has changed. Display the new value to the terminal.
                self.get_logger().info("Parameter velocity_limit has changed. The new value is: %f" % param.value)
                # The parameter change was successfully handled.
                result.successful = True
            if param.name == "robot_name" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Parameter robot_name has changed. The new value is: %s" % param.value)
                result.successful = True

        return result

def main(args=None):
    """Main function to start the ROS 2 node.

    Args:
        args (List, optional): Command-line arguments. Defaults to None.
    """

    # Initialize ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of the MinimalParameter node
    minimal_parameter_node = MinimalParameter()

    # Keep the node running and processing events.
    rclpy.spin(minimal_parameter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_parameter_node.destroy_node()

    # Shutdown ROS 2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    # Execute the main function if the script is run directly
    main()


