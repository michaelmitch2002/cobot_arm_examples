/**
 * @file minimal_cpp_parameters.cpp
 * @brief Demonstrates the basics of declaring parameters in ROS 2.
 *
 * Description: Demonstrates the basics of declaring parameters in ROS 2.
 * 
 * -------
 * Subscription Topics:
 *   None
 * -------
 * Publishing Topics:
 *   String message
 *   /topic_cpp - std_msgs/String
 * -------
 * @author Addison Sears-Collins
 * @date 2024-03-06
 */
 
#include <memory> // Include for smart pointer utilities
#include <rclcpp/rclcpp.hpp> // ROS 2 C++ client library
#include <rcl_interfaces/msg/parameter_descriptor.hpp> // Enables parameters' metadata such as their names and types.
#include <rclcpp/parameter.hpp> // Include for manipulating parameters within a node.

/**
 * @class MinimalParameter
 * @brief Defines a minimal ROS 2 node that declares parameters.
 *
 * This class inherits from rclcpp::Node and demonstrates the process of declaring
 * parameters within a ROS 2 node. It showcases how to declare parameters with
 * default values and descriptions, and how to handle parameter change requests
 * via a callback mechanism.
 */
 class MinimalParameter : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a MinimalParameter node.
     *
     * Initializes the node, declares two parameters (`velocity_limit` and `robot_name`)
     * with default values and descriptions, and registers a callback for handling
     * parameter change requests.
     */
    MinimalParameter() : Node("minimal_parameter_cpp_node")
    {
        // Describe parameters
        rcl_interfaces::msg::ParameterDescriptor velocity_limit_descriptor;
        velocity_limit_descriptor.description = "Maximum allowed angular velocity in radians per second (ignored for fixed joints)";

        rcl_interfaces::msg::ParameterDescriptor robot_name_descriptor;
        robot_name_descriptor.description = "The name of the robot";

        // Declare parameters
        this->declare_parameter("velocity_limit", 2.792527, velocity_limit_descriptor);
        this->declare_parameter("robot_name", "Automatic Addison Bot", robot_name_descriptor);
    
        // Register a callback function that will be called whenever there is an attempt 
        // to change one or more parameters of the node.  
        parameter_callback_handle = this->add_on_set_parameters_callback(
            std::bind(&MinimalParameter::parameter_change_callback, this, std::placeholders::_1));		
    }
	
private:
    /**
     * @brief Callback function for handling parameter change requests.
     *
     * This method is invoked automatically whenever there is an attempt to change
     * one or more of the node's parameters. It checks each parameter change request,
     * logs the new value of recognized parameters, and validates the change.
     *
     * @param params A vector containing the parameters attempted to be changed.
     * @return An instance of rcl_interfaces::msg::SetParametersResult indicating
     *         whether the parameter changes were successful or not.
     */

    // Member variable to store the callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;

    rcl_interfaces::msg::SetParametersResult parameter_change_callback(const std::vector<rclcpp::Parameter> &params)
    {
        // Create a result object to report the outcome of parameter changes.
	    auto result = rcl_interfaces::msg::SetParametersResult();

        // Assume success unless an unsupported parameter is encountered.
        result.successful = true;

        // Iterate through each parameter in the change request.    
        for (const auto &param : params) 
        {
            // Check if the changed parameter is 'velocity_limit' and of type double.
            if (param.get_name() == "velocity_limit" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) 
            {
                RCLCPP_INFO(this->get_logger(), "Parameter velocity_limit has changed. The new value is: %f", param.as_double());
            } 
            // Check if the changed parameter is 'robot_name' and of type string.
            else if (param.get_name() == "robot_name" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) 
            {
                RCLCPP_INFO(this->get_logger(), "Parameter robot_name has changed. The new value is: %s", param.as_string().c_str());
            } 
            // Handle any unsupported parameters.
            else 
            {
                // Mark the result as unsuccessful and provide a reason.
                result.successful = false;
                result.reason = "Unsupported parameter";
            }
        } 
        // Return the result object, indicating whether the parameter change(s) were successful or not.		
        return result;
    }
};

/**
 * @brief Main function.
 *
 * This function initializes the ROS 2 system, creates and runs an instance of the
 * MinimalParameter node. It keeps the node running until it is manually terminated,
 * ensuring that the node can react to parameter changes or perform its intended
 * functions during its lifecycle. Finally, it shuts down the ROS 2 system before
 * terminating the program.
 *
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return int Returns 0 upon successful completion of the program.
 */
int main(int argc, char * argv[])
{
    // Initialize ROS 2.
    rclcpp::init(argc, argv); 
  
    // Create an instance of the MinimalParameter node and keep it running.
    auto minimal_parameter_cpp_node = std::make_shared<MinimalParameter>();
    rclcpp::spin(minimal_parameter_cpp_node);

    // Shutdown ROS 2 upon node termination.
    rclcpp::shutdown(); 

    // End of program.
    return 0; 
}