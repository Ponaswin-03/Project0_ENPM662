/**
 * @file camera_processor.hpp
 * @author Vignesh Rajagopal (vigneshr@umd.edu)
 * @author Pon Aswin Sankaralingam (aswin03@umd.edu)
 * @author Gaurav Upadhyay (ugaurav@umd.edu)
 * @brief  Header file defining the CameraProcessor class for processing and managing parts in the environment.
 * 
 * Includes functionality for subscribing to parts topics, processing camera data,
 * transforming part poses, and broadcasting transforms in a ROS 2 environment.
 * 
 * @version 0.1
 * @date 2024-12-10
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mage_msgs/msg/parts.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace group7_final {

/**
 * @class CameraProcessor
 * @brief ROS 2 node for processing parts data and camera images.
 * 
 * This class manages part subscriptions, camera data processing, coordinate transformations,
 * and transform broadcasting to enable navigation and interaction with parts in the environment.
 */
class CameraProcessor : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the CameraProcessor class.
     */
    CameraProcessor();

private:
    /**
     * @struct ProcessedPart
     * @brief Data structure for storing processed part information.
     */
    struct ProcessedPart {
        uint8_t type; ///< Part type identifier.
        uint8_t color; ///< Part color identifier.
        geometry_msgs::msg::Pose pose; ///< Pose of the part in the world frame.
    };
    std::map<std::string, std::vector<ProcessedPart>> processed_parts_; ///< Map of processed parts by camera frame.

    /**
     * @brief Callback for processing parts data.
     * @param msg Shared pointer to the parts message.
     */
    void parts_callback(const mage_msgs::msg::Parts::SharedPtr msg);

    /**
     * @brief Callback for processing camera data.
     * @param msg Shared pointer to the camera image message.
     * @param camera_id_ Identifier of the camera.
     */
    void camera_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg, size_t camera_id_);

    /**
     * @brief Broadcasts a transform from the camera to the world frame.
     * @param camera_pose Pose of the camera.
     * @param camera_frame Frame of the camera.
     */
    void broadcast_camera_to_world_transform(const geometry_msgs::msg::Pose& camera_pose, const std::string& camera_frame);

    /**
     * @brief Transforms a part pose from one frame to another.
     * @param from_frame Source frame.
     * @param to_frame Target frame.
     * @param pose Pose to transform.
     * @return True if transformation was successful, false otherwise.
     */
    bool transform_part_pose(const std::string& from_frame, const std::string& to_frame, geometry_msgs::msg::Pose& pose);
    /**
     * @brief Processes individual parts detected in the camera image.
     * @param part Part message containing type and color.
     * @param part_pose Pose of the part in the camera frame.
     * @param camera_frame Frame of the camera.
     */
    void process_part(const mage_msgs::msg::Part& part, const geometry_msgs::msg::Pose& part_pose, const std::string& camera_frame);

    size_t camera_id_{0}; ///< Current camera being processed.
    bool parts_processed_{false}; ///< Flag indicating whether all parts have been processed.
    rclcpp::Subscription<mage_msgs::msg::Parts>::SharedPtr parts_subscription_; ///< Subscription to parts topic.
    std::vector<mage_msgs::msg::Part> parts_to_visit_; ///< List of parts to visit.

    std::vector<rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr> camera_subscribers_; ///< Subscriptions to camera topics.
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr processed_camera_processor_; ///< Publisher for processed parts poses.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; ///< Buffer for managing TF2 transforms.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< Listener for receiving TF2 transforms.
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; ///< Broadcaster for publishing TF2 transforms.
};

} // namespace group7_final