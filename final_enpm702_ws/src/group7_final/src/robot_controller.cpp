#include "robot_controller.hpp"

namespace group7_final
{
    // Constructor for the RobotController class
    RobotController::RobotController() : Node("robot_controller"),
                                         navigation_completed_(false),
                                         current_target_index_(0),
                                         kp_linear_(0.5), // Proportional gain for linear velocity
                                         kp_angular_(1.0), // Proportional gain for angular velocity
                                         max_linear_vel_(0.2), // Maximum linear velocity
                                         max_angular_vel_(0.5) // Maximum angular velocity
    {
        // Initialize publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribe to the processed parts topic to receive target poses
        processed_parts_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "processed_parts", 10, std::bind(&RobotController::processedPartsCallback, this, std::placeholders::_1));

        // Subscribe to odometry data to track the robot's current position
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RobotController::odomCallback, this, std::placeholders::_1));

        // Create a timer to periodically execute the navigation control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotController::navigateToTargets, this));

        returning_to_origin_ = false; // Flag to indicate if the robot is returning to its starting position
    }

    // Callback for receiving new target poses
    void RobotController::processedPartsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        target_poses_ = msg->poses; // Store the received poses
        navigation_completed_ = false; // Reset navigation status
        current_target_index_ = 0; // Start at the first target
    }

    // Callback for receiving current pose from odometry data
    void RobotController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose; // Update the robot's current pose
    }

    // Main navigation control loop
    void RobotController::navigateToTargets()
    {
        if (target_poses_.empty() || navigation_completed_)
        {
            moveToOrigin(); // If no targets or navigation is complete, return to origin
            return;
        }

        // Navigate to the current target pose
        const auto &target_pose = target_poses_[current_target_index_];
        moveToTarget(target_pose);
    }

    // Compute the Euclidean distance between the current and target poses
    double RobotController::computeDistance(const geometry_msgs::msg::Pose &current, const geometry_msgs::msg::Pose &target)
    {
        double dx = target.position.x - current.position.x;
        double dy = target.position.y - current.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Compute the angle needed to steer towards the target pose
    double RobotController::computeSteeringAngle(const geometry_msgs::msg::Pose &current, const geometry_msgs::msg::Pose &target)
    {
        double dx = target.position.x - current.position.x;
        double dy = target.position.y - current.position.y;

        double angle_to_target = std::atan2(dy, dx); // Angle to target

        // Extract the robot's current yaw from its orientation
        tf2::Quaternion q(
            current.orientation.x,
            current.orientation.y,
            current.orientation.z,
            current.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double angle = angle_to_target - yaw;

        // Normalize the angle to the range [-π, π]
        if (angle > M_PI)
            angle -= 2 * M_PI;
        else if (angle < -M_PI)
            angle += 2 * M_PI;

        return angle;
    }

    // Navigate the robot to the specified target pose
    void RobotController::moveToTarget(const geometry_msgs::msg::Pose &target_pose)
    {
        geometry_msgs::msg::Twist cmd_vel;

        double distance = computeDistance(current_pose_, target_pose); // Compute distance to target
        double angle = computeSteeringAngle(current_pose_, target_pose); // Compute steering angle

        // Compute velocities using proportional control
        double linear_vel = kp_linear_ * distance;
        double angular_vel = kp_angular_ * angle;

        // Limit velocities to their maximum values
        linear_vel = std::clamp(linear_vel, -max_linear_vel_, max_linear_vel_);
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

        // Publish velocity commands
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd_vel);

        // Check if the target has been reached
        if (distance < 0.1) 
        {
            RCLCPP_INFO(this->get_logger(), "Reached target %zu", current_target_index_);
            current_target_index_++;
            if (current_target_index_ >= target_poses_.size())
            {
                navigation_completed_ = true; // All targets reached
                returning_to_origin_ = true; // Start returning to origin
                RCLCPP_INFO(this->get_logger(), "Navigation completed, Returning to base");
                // rclcpp::shutdown();
            }
        }
    }

    // Move the robot back to its starting position
    void RobotController::moveToOrigin()
    {
        geometry_msgs::msg::Pose origin_pose;
        origin_pose.position.x = 0.0;
        origin_pose.position.y = 0.0;
        origin_pose.position.z = 0.0;
        origin_pose.orientation.w = 1.0; // Identity quaternion

        double distance = computeDistance(current_pose_, origin_pose);
        double angle = computeSteeringAngle(current_pose_, origin_pose);

        geometry_msgs::msg::Twist cmd_vel;

        if (distance > 0.1) // If not close to origin
        {
            // Compute velocities to move to origin
            double linear_vel = kp_linear_ * distance;
            double angular_vel = kp_angular_ * angle;

            linear_vel = std::clamp(linear_vel, -max_linear_vel_, max_linear_vel_);
            angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

            cmd_vel.linear.x = linear_vel;
            cmd_vel.angular.z = angular_vel;
            cmd_vel_pub_->publish(cmd_vel);
        }
        else
        {
            // Stop the robot when close enough
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);

            returning_to_origin_ = false; // Reset flag
        }
    }
} // namespace group7_final
