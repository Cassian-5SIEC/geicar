#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

using namespace BT;
class CheckTargetInRange : public StatefulActionNode {
    public:
        CheckTargetInRange(const std::string& name, const NodeConfig& config, const std::shared_ptr<rclcpp::Node> nh)
                        : StatefulActionNode(name, config), node_(nh)
        {
            // Initialize TF2 buffer and listener
            tf_target_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_target_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_target_buffer_);
        }
        static PortsList providedPorts() 
        {
            return {};
        }


        NodeStatus onStart() override {
            isRunning_ = true;
            execSuccess_ = false;
            exec_thread_ = std::make_shared<std::thread>(std::bind(&CheckTargetInRange::checkTarget, 
                                                            this, 
                                                            std::ref(isRunning_),
                                                            std::ref(execSuccess_)));
            return NodeStatus::RUNNING;
        }
        NodeStatus onRunning() override {
            if (isRunning_) {
                return NodeStatus::RUNNING;
            }
            exec_thread_->join();
            if (execSuccess_) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        }

        void onHalted() override {
            isRunning_ = false;
            exec_thread_->detach();
        }

    private:

        void checkTarget(std::atomic<bool>& isRunning, std::atomic<bool>& execSuccess){
            geometry_msgs::msg::PoseStamped target_pose = geometry_msgs::msg::PoseStamped();
            // Set target pose at the origin of the "target_trash" frame
            target_pose.header.frame_id = "target_trash";  
            target_pose.header.stamp = rclcpp::Time(0);
            target_pose.pose.orientation.w = 1.0;
            
            try {
                // Transform the target pose to the Shoulder_Rotation_Pitch frame using TF2
                target_pose = tf_target_buffer_->transform(target_pose, "Arm_Base");

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "Could not transform pose: %s", ex.what());
                execSuccess = false;
                isRunning = false;
                return;
            }
            double dx = target_pose.pose.position.x;;
            double dy = target_pose.pose.position.y;;
            double distance = std::sqrt(dx*dx + dy*dy);
            if (distance > ARM_MAX_RANGE || distance < ARM_MIN_RANGE){
                RCLCPP_WARN(node_->get_logger(), "Target is out of range: %.3f meters", distance);
                execSuccess = false;
                isRunning = false;
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Target is within range: %.3f meters", distance);
            isRunning = false;
            execSuccess = true;
            return;
        } 

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<std::thread> exec_thread_;
        std::atomic<bool> isRunning_;
        std::atomic<bool> planSuccess_;
        std::atomic<bool> execSuccess_;

        // TF2 buffer and listener
        std::unique_ptr<tf2_ros::Buffer> tf_target_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_target_listener_{nullptr};

        const double ARM_MAX_RANGE = 0.42;
        const double ARM_MIN_RANGE = 0.30;
};