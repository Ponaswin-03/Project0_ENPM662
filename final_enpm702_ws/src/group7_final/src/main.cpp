#include <rclcpp/rclcpp.hpp>
#include "camera_processor.hpp"
#include "robot_controller.hpp"

/**
 * @mainpage  
 *
 * @section intro_sec Introduction
 * 
 * *This program demonstrates the integration of ROS 2 nodes for the turtlebot to navigate autonomously
 * and process the parts within a simulated Gazebo environment.
 *
 * @section usage_sec Usage
 *
 * Build the package - colcon build --packages-select <package_name>
 * Source the workspace
 * Run the robot Gazebo with a emplt world eniviroment - ros2 launch final_project final_project.launch.py
 * run the node - ros2 run group7_final camera_processor
 */

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Create multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // Create both nodes
    auto publisher_node = std::make_shared<group7_final::CameraProcessor>();
    auto controller_node = std::make_shared<group7_final::RobotController>();
    
    // Add nodes to executor
    executor.add_node(publisher_node);
    executor.add_node(controller_node);
    
    // Spin both nodes
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}