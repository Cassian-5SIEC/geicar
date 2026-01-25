#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include <thread>
#include <atomic>
#include <chrono>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace BT;

class SetGoalOffset : public StatefulActionNode {
    public:
        SetGoalOffset(const std::string& name, const NodeConfig& config, const std::shared_ptr<rclcpp::Node> nh)
                        : StatefulActionNode(name, config), node_(nh)
        {
            // Initialize TF2 buffer and listener
            tf_target_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_target_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_target_buffer_);

            marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/goal_debug_markers", 10);
        }

        static PortsList providedPorts() 
        {
            return {
                InputPort<std::string>("target_frame", "target_trash", "The reference frame for the target pose"),
                InputPort<double>("standoff_distance", 0.5, "Distance to stand off from the target"),
                InputPort<double>("angle_offset_deg", 0.0, "Angle offset in degrees (Neg=Left, Pos=Right)"),
                OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "The offset goal pose for the mobile base")
            };
        }

        NodeStatus onStart() override {
            isRunning_ = true;
            execSuccess_ = false;
            
            // Start the worker thread
            exec_thread_ = std::make_shared<std::thread>(std::bind(&SetGoalOffset::setGoalOffset, this));
            
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override {
            // If the thread is still working, keep running
            if (isRunning_) {
                return NodeStatus::RUNNING;
            }
            
            // Thread finished (isRunning_ is false). Join it safely.
            if (exec_thread_->joinable()) {
                exec_thread_->join();
            }

            if (execSuccess_) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        }

        void onHalted() override {
            // Signal the thread to stop
            isRunning_ = false;
            
            if (exec_thread_ && exec_thread_->joinable()) {
                exec_thread_->join();
            }
        }

private:

        void setGoalOffset() {
            // Get Inputs
            Expected<std::string> target_frame_ex = getInput<std::string>("target_frame");
            Expected<double> standoff_dist_ex = getInput<double>("standoff_distance");
            Expected<double> angle_offset_deg_ex = getInput<double>("angle_offset_deg");

            if (!target_frame_ex || !standoff_dist_ex || !angle_offset_deg_ex) {
                RCLCPP_ERROR(node_->get_logger(), "SetGoalOffset: Missing input parameters");
                execSuccess_ = false;
                isRunning_ = false;
                return;
            }

            std::string target_frame = target_frame_ex.value();
            double standoff = standoff_dist_ex.value();
            // Convert Degrees to Radians for math
            double angle_offset_rad = angle_offset_deg_ex.value() * (M_PI / 180.0);

            // Wait for Transform (Target -> Map)
            std::string fixed_frame = "map"; 
            if (!waitForTransform(fixed_frame, target_frame)) return;

            try {
                // Get Target Position in Map
                // Create a zero-pose in the target frame and transform it to map
                geometry_msgs::msg::PoseStamped target_pose_in_frame;
                target_pose_in_frame.header.frame_id = target_frame;
                target_pose_in_frame.header.stamp = rclcpp::Time(0);
                target_pose_in_frame.pose.orientation.w = 1.0;

                geometry_msgs::msg::PoseStamped target_in_map;
                target_in_map = tf_target_buffer_->transform(target_pose_in_frame, fixed_frame);

                // Get Robot Position in Map (to calculate base approach angle)
                geometry_msgs::msg::TransformStamped map_to_robot;
                map_to_robot = tf_target_buffer_->lookupTransform(fixed_frame, "base_link", rclcpp::Time(0));

                double robot_x = map_to_robot.transform.translation.x;
                double robot_y = map_to_robot.transform.translation.y;
                double target_x = target_in_map.pose.position.x;
                double target_y = target_in_map.pose.position.y;

                // Calculate Base Angle (Theta)
                // This is the angle pointing FROM Robot TO Target
                double theta = std::atan2(target_y - robot_y, target_x - robot_x);

                // Apply Angle Offset
                // We simply shift the approach angle
                double final_approach_angle = theta + angle_offset_rad;

                // Calculate Goal Position using Polar Coordinates
                // Start at Target, move BACKWARDS along the final angle by 'standoff'
                // Goal = Target - (Standoff * UnitVector(angle))
                target_in_map.pose.position.x -= (standoff * cos(theta));
                target_in_map.pose.position.y -= (standoff * sin(theta));
                target_in_map.pose.position.z = 0.0;

                // Set Orientation: Face the target with an offset
                // The robot should face along the approach angle
                tf2::Quaternion q;
                q.setRPY(0, 0, final_approach_angle);
                target_in_map.pose.orientation = tf2::toMsg(q);

                setOutput("goal_pose", target_in_map);
                
                RCLCPP_INFO(node_->get_logger(), "Offset Goal: %.2fm dist, %.1f deg offset", standoff, angle_offset_deg_ex.value());
                drawMarkerAtPose(target_in_map, target_frame);
                execSuccess_ = true;

            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "SetGoalOffset TF Error: %s", ex.what());
                execSuccess_ = false;
            }

            isRunning_ = false;
        }

        // Helper to keep the code clean and prevent infinite loops
        bool waitForTransform(const std::string& target, const std::string& source) {
            for (int i = 0; i < 10; i++) {
                if (!isRunning_) return false; // Safety check
                if (tf_target_buffer_->canTransform(target, source, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
                    return true;
                }
            }
            RCLCPP_WARN(node_->get_logger(), "Timeout waiting for transform %s -> %s", source.c_str(), target.c_str());
            return false;
        }

        void drawMarkerAtPose(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame) {
            // --- VISUALIZATION START ---
            visualization_msgs::msg::MarkerArray marker_array;

            // Marker 1: The Arrow (Pose)
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = node_->now(); // Use current time for visualization
            arrow.ns = "goal_arrow";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = pose.pose; // The calculated goal pose
            arrow.scale.x = 0.3; // Length
            arrow.scale.y = 0.05; // Width
            arrow.scale.z = 0.05; // Height
            arrow.color.r = 0.0;
            arrow.color.g = 1.0; // Green
            arrow.color.b = 0.0;
            arrow.color.a = 1.0; // Alpha (transparency)

            // Marker 2: The Text Label
            visualization_msgs::msg::Marker text;
            text.header.frame_id = "map";
            text.header.stamp = node_->now();
            text.ns = "goal_text";
            text.id = 1;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose = pose.pose;
            text.pose.position.z += 0.3; // Floating 30cm above the arrow
            text.scale.z = 0.2; // Text Height
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            text.text = "Offset Goal " + target_frame;

            marker_array.markers.push_back(arrow);
            marker_array.markers.push_back(text);

            marker_pub_->publish(marker_array);
            // --- VISUALIZATION END ---
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<std::thread> exec_thread_;
        std::atomic<bool> isRunning_;
        std::atomic<bool> execSuccess_;

        std::unique_ptr<tf2_ros::Buffer> tf_target_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_target_listener_{nullptr};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};