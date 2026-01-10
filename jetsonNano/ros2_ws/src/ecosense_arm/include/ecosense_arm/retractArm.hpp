#include "moveit/move_group_interface/move_group_interface.h"

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_ros2/bt_action_node.hpp>

using namespace BT;
class RetractArm : public StatefulActionNode {
    public:
        RetractArm(const std::string& name, const NodeConfig& config, const std::shared_ptr<rclcpp::Node> nh)
                        : StatefulActionNode(name, config), node_(nh)
        {
            // Create the MoveIt MoveGroup Interface
            using moveit::planning_interface::MoveGroupInterface;
            arm_group_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            gripper_group_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
            arm_group_->setPoseReferenceFrame("Arm_Base");
            gripper_group_->setPoseReferenceFrame("Arm_Base");
        }
        static PortsList providedPorts() 
        {
            return {};
        }


        NodeStatus onStart() override {
            isRunning_ = true;
            execSuccess_ = false;
            exec_thread_ = std::make_shared<std::thread>(std::bind(&RetractArm::execute, 
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
        void execute(std::atomic<bool>& isRunning, std::atomic<bool>& execSuccess){
            gripper_group_->setNamedTarget("open_gripper");
            gripper_group_->move();
            gripper_group_->setNamedTarget("close_gripper");
            gripper_group_->move();

            // Move to home pose
            arm_group_->setNamedTarget("home");
            if (arm_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(node_->get_logger(), "Failed to move to home pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }
            isRunning = false;
            execSuccess = true;
            return;
        }

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<std::thread> exec_thread_;
        std::atomic<bool> isRunning_;
        std::atomic<bool> execSuccess_;
};