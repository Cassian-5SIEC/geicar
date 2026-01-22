#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

using namespace BT;
class SetGoalOffset : public StatefulActionNode {
    public:
        SetGoalOffset(const std::string& name, const NodeConfig& config, const std::shared_ptr<rclcpp::Node> nh)
                        : StatefulActionNode(name, config), node_(nh)
        {
            // Initialize TF2 buffer and listener
            tf_target_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_target_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_target_buffer_);
        }
        static PortsList providedPorts() 
        {
            return {InputPort<std::string>("target_frame", "target_trash", "The reference frame for the target pose"),
                    InputPort<double>("standoff_distance", 0.5, "Distance to stand off from the target"),
                    OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "The offset goal pose for the mobile base")
                    };
        }


        NodeStatus onStart() override {
            isRunning_ = true;
            execSuccess_ = false;
            exec_thread_ = std::make_shared<std::thread>(std::bind(&SetGoalOffset::setGoalOffset, 
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

        void setGoalOffset(std::atomic<bool>& isRunning, std::atomic<bool>& execSuccess){
            Expected<std::string> target_frame_ = getInput<std::string>("target_frame");
            Expected<double> standoff_distance_ = getInput<double>("standoff_distance");
            if (!target_frame_ || !standoff_distance_) {
                RCLCPP_ERROR(node_->get_logger(), "SetGoalOffset: Missing input parameters");
                execSuccess = false;
                isRunning = false;
                return;
            }
            geometry_msgs::msg::PoseStamped nav_goal_pose = geometry_msgs::msg::PoseStamped();
            // Set target pose at the origin of the "target_frame" frame
            nav_goal_pose.header.frame_id = target_frame_.value();  
            nav_goal_pose.header.stamp = rclcpp::Time(0);
            nav_goal_pose.pose.orientation.w = 1.0;
            
            try {
                // Transform the target pose to the Shoulder_Rotation_Pitch frame using TF2
                nav_goal_pose = tf_target_buffer_->transform(nav_goal_pose, "base_link");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "Could not transform pose: %s", ex.what());
                execSuccess = false;
                isRunning = false;
                return;
            }
            // Rotate the goal pose to face away from base_link
            double yaw_angle = std::atan2(nav_goal_pose.pose.position.y, nav_goal_pose.pose.position.x);
            tf2::Quaternion tf_quat;
            tf_quat.setRPY(0.0, 0.0, yaw_angle);
            nav_goal_pose.pose.orientation = tf2::toMsg(tf_quat);
            // Apply the standoff distance
            double distance = std::sqrt(
                nav_goal_pose.pose.position.x * nav_goal_pose.pose.position.x +
                nav_goal_pose.pose.position.y * nav_goal_pose.pose.position.y
            );

            if (distance < 1e-3){
                RCLCPP_WARN(node_->get_logger(), "Target is too close to base_link to apply standoff.");
                execSuccess = false;
                isRunning = false;
                return;
            }

            nav_goal_pose.pose.position.x -= (standoff_distance_.value() * cos(yaw_angle));
            nav_goal_pose.pose.position.y -= (standoff_distance_.value() * sin(yaw_angle));
            nav_goal_pose.pose.position.z = 0.0; // Keep goal on the ground plane

            // Set the output port
            setOutput<geometry_msgs::msg::PoseStamped>("goal_pose", nav_goal_pose);
            RCLCPP_INFO(node_->get_logger(), "SetGoalOffset: Goal pose set at (%.2f, %.2f, %.2f) in base_link frame.",
                        nav_goal_pose.pose.position.x,
                        nav_goal_pose.pose.position.y,
                        nav_goal_pose.pose.position.z);
            execSuccess = true;
            isRunning = false;
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
};