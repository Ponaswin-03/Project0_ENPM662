/**
 * @file robot_controller.hpp
 * @author Vignesh Rajagopal (vigneshr@umd.edu)
 * @author Pon Aswin Sankaralingam (aswin03@umd.edu)
 * @author Gaurav Upadhyay (ugaurav@umd.edu)
 * @brief Header file defining the RobotController class for TurtleBot navigation and control.
 *
 * This class provides functionality for navigating a robot to multiple target locations
 * using proportional control, while processing odometry and pose data in a ROS 2 environment.
 * @version 0.1
 * @date 2024-12-10
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>
#include <chrono>

namespace group7_final {

/**
 * @class RobotController
 * @brief ROS 2 node for controlling and navigating a TurtleBot in a simulated environment.
 *
 * The RobotController class manages navigation to multiple targets using proportional
 * control for linear and angular velocities, while processing odometry and pose data.
 */
class RobotController : public rclcpp::Node {
public:
    /**
     * @brief Constructs a new RobotController object.
     */
    RobotController();

private:
    // Publishers and Subscribers

    /**
     * @brief Publisher for velocity commands.
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    /**
     * @brief Subscriber for processed parts (target poses).
     */
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr processed_parts_subscriber_;

    /**
     * @brief Subscriber for odometry data.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    /**
     * @brief Timer for periodic navigation updates.
     */
    rclcpp::TimerBase::SharedPtr timer_;

    // Navigation Variables

    /**
     * @brief List of target poses to navigate to.
     */
    std::vector<geometry_msgs::msg::Pose> target_poses_;

    /**
     * @brief Current pose of the robot.
     */
    geometry_msgs::msg::Pose current_pose_;

    /**
     * @brief Flag indicating whether navigation to all targets is complete.
     */
    bool navigation_completed_;

    /**
     * @brief Index of the current target pose in the navigation sequence.
     */
    size_t current_target_index_;

    // Proportional Control Parameters

    /**
     * @brief Proportional gain for linear velocity control.
     */
    double kp_linear_;

    /**
     * @brief Proportional gain for angular velocity control.
     */
    double kp_angular_;

    /**
     * @brief Maximum allowable linear velocity.
     */
    double max_linear_vel_;

    /**
     * @brief Maximum allowable angular velocity.
     */
    double max_angular_vel_;

    bool returning_to_origin_;

    // Callbacks

    /**
     * @brief Callback to process a new set of target poses.
     * @param msg Shared pointer to the PoseArray message containing target poses.
     */
    void processedPartsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    /**
     * @brief Callback to update the robot's current pose based on odometry data.
     * @param msg Shared pointer to the Odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Periodically navigates the robot to the next target pose.
     */
    void navigateToTargets();

    // Helper Functions

    /**
     * @brief Computes the Euclidean distance between the current and target poses.
     * @param current The robot's current pose.
     * @param target The target pose.
     * @return The distance in meters.
     */
    double computeDistance(const geometry_msgs::msg::Pose &current, const geometry_msgs::msg::Pose &target);

    /**
     * @brief Computes the steering angle required to face the target pose.
     * @param current The robot's current pose.
     * @param target The target pose.
     * @return The angle in radians.
     */
    double computeSteeringAngle(const geometry_msgs::msg::Pose &current, const geometry_msgs::msg::Pose &target);

    /**
     * @brief Moves the robot towards a specified target pose.
     * @param target_pose The target pose to navigate to.
     */
    void moveToTarget(const geometry_msgs::msg::Pose &target_pose);

    void moveToOrigin();
};

} // namespace group7_final