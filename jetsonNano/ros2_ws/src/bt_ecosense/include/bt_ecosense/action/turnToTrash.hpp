#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_ros2/bt_action_node.hpp>

using namespace BT;
class TurnToTrash : public StatefulActionNode {
    public:
        TurnToTrash(const std::string& name, const NodeConfig& config, const std::shared_ptr<rclcpp::Node> nh)
                        : StatefulActionNode(name, config), node_(nh)
        {
            // Initialize TF2 buffer and listener
            tf_target_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_target_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_target_buffer_);
        }
        static PortsList providedPorts() 
        {
            return {
                InputPort<double>("trash_angle", 0.0, "Angle of trash in LiDAR frame"),
            };
        }


        NodeStatus onStart() override {
            isRunning_ = true;
            execSuccess_ = false;
            current_trash_angle_ = getInput<double>("trash_angle").value();
            exec_thread_ = std::make_shared<std::thread>(std::bind(&TurnToTrash::execute, 
                                                            this, 
                                                            std::ref(isRunning_),
                                                            std::ref(execSuccess_)));
            return NodeStatus::RUNNING;
        }
        NodeStatus onRunning() override {
            if (isRunning_) {
                current_trash_angle_ = getInput<double>("trash_angle").value();
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
        
        void execute(std::atomic<bool>& isRunning, std::atomic<bool>& execSuccess){
            auto vel_msg = geometry_msgs::msg::Twist();

            if (current_trash_angle_ > M_PI/2){

            }

            isRunning = true;
            execSuccess = false;
            return;
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<std::thread> exec_thread_;
        std::atomic<bool> isRunning_;
        std::atomic<bool> planSuccess_;
        std::atomic<bool> execSuccess_;

        double current_trash_angle_ = 0.0;

        // Publisher for velocity commands
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};