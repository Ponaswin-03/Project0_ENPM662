#include "camera_processor.hpp"

namespace group7_final {
    // Constructor for CameraProcessor class, initializes all necessary components.
    CameraProcessor::CameraProcessor() : Node("camera_processor") {
        // Create subscription to receive parts information
        parts_subscription_ = this->create_subscription<mage_msgs::msg::Parts>(
            "/parts", 10, std::bind(&CameraProcessor::parts_callback, this, std::placeholders::_1));

        // Initialize the TF buffer and listener for transform operations
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize 8 camera subscriptions, one for each camera
        camera_subscribers_.resize(8);
        for (size_t i = 0; i < 8; ++i) {
            std::string topic_name = "/mage/camera" + std::to_string(i + 1) + "/image";
            auto callback = [this, i](const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
                this->camera_callback(msg, i);
            };
            
            // Create subscription for each camera image topic
            camera_subscribers_[i] = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                topic_name,
                rclcpp::SensorDataQoS(),
                callback
            );
        }
        // Publisher for processed parts to be shared with other nodes
        processed_camera_processor_ = this->create_publisher<geometry_msgs::msg::PoseArray>("processed_parts", 10);
    }
    
    // Callback to receive parts information from the parts topic
    void CameraProcessor::parts_callback(const mage_msgs::msg::Parts::SharedPtr msg)
    {
        if (!parts_processed_)
        {
            // RCLCPP_INFO(this->get_logger(), "Parts list received:");
            parts_to_visit_ = msg->parts;
            for (const auto& part : parts_to_visit_)
            {
                RCLCPP_INFO(this->get_logger(), "Color: %d, Type: %d", part.color, part.type);
            }
            parts_processed_ = true;  // Set flag to true once parts are processed
        }
    }

    // Callback function that handles camera data, processes parts information
    void CameraProcessor::camera_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg, size_t camera_id_)
    {
        std::string camera_frame = "camera_" + std::to_string(camera_id_ + 1);

        // Broadcast the transform from the camera to the world
        broadcast_camera_to_world_transform(msg->sensor_pose, camera_frame);

        // Process each part detected by the camera
        for (const auto& part : msg->part_poses)
        {
            // Broadcast the transform from camera to part
            geometry_msgs::msg::TransformStamped part_transform;
            part_transform.header.stamp = this->get_clock()->now();
            part_transform.header.frame_id = camera_frame;
            part_transform.child_frame_id = "part_" + std::to_string(camera_id_) + "_" + std::to_string(part.part.color) + "_" + std::to_string(part.part.type);

            // Set the part's position and orientation in the transform
            part_transform.transform.translation.x = part.pose.position.x;
            part_transform.transform.translation.y = part.pose.position.y;
            part_transform.transform.translation.z = part.pose.position.z;
            part_transform.transform.rotation = part.pose.orientation;

            // Send the transform to the TF broadcaster
            tf_broadcaster_->sendTransform(part_transform);

            // Process the part
            process_part(part.part, part.pose, camera_frame);
        }
    }

    // Broadcasts the transform from camera frame to the world frame
    void CameraProcessor::broadcast_camera_to_world_transform(const geometry_msgs::msg::Pose& camera_pose, const std::string& camera_frame)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = camera_frame;

        t.transform.translation.x = camera_pose.position.x;
        t.transform.translation.y = camera_pose.position.y;
        t.transform.translation.z = camera_pose.position.z;
        t.transform.rotation = camera_pose.orientation;

        // Send the camera to world transform
        tf_broadcaster_->sendTransform(t);
    }

    // Transform a part's pose from one frame to another
    bool CameraProcessor::transform_part_pose(const std::string& from_frame, const std::string& to_frame, geometry_msgs::msg::Pose& pose)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            // Look up the transformation from the TF buffer
            transform_stamped = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", from_frame.c_str(), to_frame.c_str(), ex.what());
            return false;
        }

        // Apply the transformation to the pose
        tf2::doTransform(pose, pose, transform_stamped);
        return true;
    }

    // Process a detected part by transforming its pose to the world frame
    void CameraProcessor::process_part(const mage_msgs::msg::Part& part, const geometry_msgs::msg::Pose& part_pose, const std::string& camera_frame)
    {
        geometry_msgs::msg::Pose world_pose = part_pose;
        // Transform part pose from camera frame to world frame
        if (transform_part_pose(camera_frame, "world", world_pose))
        {
            auto it = std::find_if(parts_to_visit_.begin(), parts_to_visit_.end(),
                [&part](const mage_msgs::msg::Part& p){
                    return p.color==part.color&& p.type ==part.type;
                });
            if(it != parts_to_visit_.end()){
            // Create a unique key for the part based on its color and type
            std::string part_key = std::to_string(part.color) + "_" + std::to_string(part.type);
            processed_parts_[part_key].push_back({part.type, part.color, world_pose});

            // Log information about processed parts
            RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Processed Parts:");
            RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------");
            
            geometry_msgs::msg::PoseArray pose_array;
            pose_array.header.stamp = this->now();
            pose_array.header.frame_id = "world";

            // Log and prepare pose array for publishing
            for (const auto& [part_key, instances] : processed_parts_)
            {
                size_t delimiter_pos = part_key.find('_');
                std::string color = part_key.substr(0, delimiter_pos);
                std::string type = part_key.substr(delimiter_pos + 1);

                
                for (size_t i = 0; i < instances.size(); ++i)
                {
                    const auto& instance = instances[i];
                    RCLCPP_INFO(this->get_logger(), "  Instance %zu: position: x=%f, y=%f, z=%f",
                                i + 1, instance.pose.position.x, instance.pose.position.y, instance.pose.position.z);
                    pose_array.poses.push_back(instance.pose);
                    RCLCPP_INFO(this->get_logger(), "Part: Color %s, Type %s", color.c_str(), type.c_str());
                }
            }
            RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------");

            // Publish the processed pose array
            processed_camera_processor_->publish(pose_array);

            // If all required parts have been processed, stop listening to camera topics
            if (processed_parts_.size() == parts_to_visit_.size())
            {
                for (auto& sub : camera_subscribers_)
                {
                    sub.reset();
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Ignored, not in the enviroment");
        }
    }

    }
}

