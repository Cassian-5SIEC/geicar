#include "moveit/move_group_interface/move_group_interface.h"

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

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_ros2/bt_action_node.hpp>

using namespace BT;
class ExecutePickPlace : public StatefulActionNode {
    public:
        ExecutePickPlace(const std::string& name, const NodeConfig& config, const std::shared_ptr<rclcpp::Node> nh)
                        : StatefulActionNode(name, config), node_(nh)
        {
            // Create the MoveIt MoveGroup Interface
            using moveit::planning_interface::MoveGroupInterface;
            arm_group_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            gripper_group_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
            arm_group_->setPoseReferenceFrame("Arm_Base");
            gripper_group_->setPoseReferenceFrame("Arm_Base");
            arm_group_->setMaxVelocityScalingFactor(1.0);
            arm_group_->setMaxAccelerationScalingFactor(1.0);
            gripper_group_->setMaxVelocityScalingFactor(1.0);
            gripper_group_->setMaxAccelerationScalingFactor(1.0);
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
            exec_thread_ = std::make_shared<std::thread>(std::bind(&ExecutePickPlace::execute, 
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
        // void plan(std::atomic<bool>& isRunning, std::atomic<bool>& planSuccess,
        //             moveit::planning_interface::MoveGroupInterface::Plan& finalPlan) {
        //     std::map<std::string, double> target_joints = arm_group_->getNamedTargetValues("default_pickup_ready");
        //     moveit::core::RobotState start_state(*arm_group_->getCurrentState());
        //     start_state.setJointGroupPositions("arm", target_joints);
            
        //     double approach_angle = computeApproachAngle();
        //     if (approach_angle != NAN) {
        //         target_joints["Shoulder_Rotation"] = approach_angle;
        //         arm_group_->setJointValueTarget(target_joints);
        //         auto const [success, plan] = [this] {
        //             moveit::planning_interface::MoveGroupInterface::Plan msg;
        //             auto const ok = static_cast<bool>(arm_group_->plan(msg));
        //             return std::make_pair(ok, msg);
        //         }();
        //         planSuccess = success;
        //         finalPlan = plan;
        //     } else {
        //         planSuccess = false;
        //         // Approach angle out of reach
        //         RCLCPP_WARN(node_->get_logger(), "Target out of reach for the arm.");
        //     }
        //     isRunning = false;
        // }

        void execute(std::atomic<bool>& isRunning, std::atomic<bool>& execSuccess){
            geometry_msgs::msg::PoseStamped ready_pose;
            geometry_msgs::msg::PoseStamped target_pose;
            // Compute ready pose
            if (setReadyPose() == -1) {
                RCLCPP_WARN(node_->get_logger(), "Failed to compute ready pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan my_plan_ready;
            if (arm_group_->plan(my_plan_ready) != moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_WARN(node_->get_logger(), "Failed to plan to ready pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }

            // Execute plan to ready pose
            if (arm_group_->execute(my_plan_ready) != moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_WARN(node_->get_logger(), "Failed to execute plan to ready pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }

            gripper_group_->setNamedTarget("open_gripper");
            gripper_group_->move();

            // Compute target pose
            if (getTargetPose(target_pose, gripper_approach_offset_) == -1) {
                RCLCPP_WARN(node_->get_logger(), "Failed to compute target pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }
            // Move to target pose
            arm_group_->setPoseTarget(target_pose.pose);
            if (arm_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(node_->get_logger(), "Failed to move to target pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }

            gripper_group_->setNamedTarget("close_gripper");
            gripper_group_->move();

            // Compute lift pose
            if (setLiftPose() == -1) {
                RCLCPP_WARN(node_->get_logger(), "Failed to compute lift pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan my_plan_lift;
            if (arm_group_->plan(my_plan_lift) != moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_WARN(node_->get_logger(), "Failed to plan to lift pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }

            // Execute plan to lift pose
            if (arm_group_->execute(my_plan_lift) != moveit::core::MoveItErrorCode::SUCCESS){
                RCLCPP_WARN(node_->get_logger(), "Failed to execute plan to lift pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }
            // Move to dropping pose
            arm_group_->setNamedTarget("dropping");
            if (arm_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(node_->get_logger(), "Failed to move to dropping pose.");
                execSuccess = false;
                isRunning = false;
                return;
            }

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

        // int getTargetPose(geometry_msgs::msg::PoseStamped& pose){
        //     pose = geometry_msgs::msg::PoseStamped();
        //     // Set target pose at the origin of the "target_trash" frame
        //     pose.header.frame_id = "target_trash";  
        //     pose.header.stamp = rclcpp::Time(0);
        //     pose.pose.orientation.w = 1.0;
            
        //     try {
        //         // Transform the target pose to the Shoulder_Rotation_Pitch frame using TF2
        //         pose = tf_target_buffer_->transform(pose, "Shoulder_Rotation_Pitch");
        //         // Turn Gripper slighly to the left to account for gripper unsymetrical 
        //         pose.pose.position.x += gripper_asym_offset_;
        //         // Rotate to point Y axis to origin of Shoulder_Rotation_Pitch (lack of 1 DOF)
        //         pose.pose.orientation = rotateYToOrigin(pose.pose.position);
        //         // Transform the pose to the Arm_Base frame
        //         pose = tf_target_buffer_->transform(pose, "Arm_Base");
        //     } catch (tf2::TransformException &ex) {
        //         RCLCPP_WARN(node_->get_logger(), "Could not transform pose: %s", ex.what());
        //         return -1;
        //     }
        //     return 0;
        // }

        int setReadyPose(){
            geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
            // Set target pose at the origin of the "target_trash" frame
            pose.header.frame_id = "target_trash";  
            pose.header.stamp = rclcpp::Time(0);
            pose.pose.orientation.w = 1.0;
            
            try {
                // Transform the target pose to the Arm_Base frame using TF2
                pose = tf_target_buffer_->transform(pose, "Arm_Base");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "Could not transform pose: %s", ex.what());
                return -1;
            }
            // Compute angle from Arm_Base origin to target in XY plane
            pose.pose.position.y += gripper_asym_offset_; // Adjust for gripper asymmetry
            double target_angle = atan2(pose.pose.position.y, pose.pose.position.x);
            // Get default ready pose
            std::map<std::string, double> target_joints = arm_group_->getNamedTargetValues("default_pickup_ready");
            target_joints["Shoulder_Rotation"] = target_angle;
            arm_group_->setJointValueTarget(target_joints);
            return 0;
        }

        int setLiftPose(){
            geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
            // Set target pose at the origin of the "target_trash" frame
            pose.header.frame_id = "target_trash";  
            pose.header.stamp = rclcpp::Time(0);
            pose.pose.orientation.w = 1.0;
            
            try {
                // Transform the target pose to the Arm_Base frame using TF2
                pose = tf_target_buffer_->transform(pose, "Arm_Base");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "Could not transform pose: %s", ex.what());
                return -1;
            }
            // Compute angle from Arm_Base origin to target in XY plane
            pose.pose.position.y += gripper_asym_offset_; // Adjust for gripper asymmetry
            double target_angle = atan2(pose.pose.position.y, pose.pose.position.x);
            // Get default ready pose
            std::map<std::string, double> target_joints = arm_group_->getNamedTargetValues("dropping");
            target_joints["Shoulder_Rotation"] = target_angle;
            arm_group_->setJointValueTarget(target_joints);
            return 0;
        }

        int getTargetPose(geometry_msgs::msg::PoseStamped& pose, const double standoff_dist = 0.15){
            pose = geometry_msgs::msg::PoseStamped();
            // Set target pose at the origin of the "target_trash" frame
            pose.header.frame_id = "target_trash";  
            pose.header.stamp = rclcpp::Time(0);
            pose.pose.orientation.w = 1.0;
            
            try {
                // Transform the target pose to the Shoulder_Rotation_Pitch frame using TF2
                pose = tf_target_buffer_->transform(pose, "Shoulder_Rotation_Pitch");
                // Turn Gripper slighly to the left to account for gripper unsymetrical 
                pose.pose.position.x += gripper_asym_offset_ - 0.01;
                // Rotate to point Y axis to origin of Shoulder_Rotation_Pitch (lack of 1 DOF)
                pose.pose.orientation = rotateYToOrigin(pose.pose.position);
                // Back off along the approach vector by standoff_dist
                double dz = pose.pose.position.z;
                double dx = pose.pose.position.x;
                double distance = std::sqrt(dx*dx + dz*dz);

                if (distance < 1e-3) return -1; // Too close to origin

                double uz = dz / distance;
                double ux = dx / distance;
                pose.pose.position.z -= (uz * standoff_dist);
                pose.pose.position.x -= (ux * standoff_dist);

                // Finally, transform the pose to the Arm_Base frame
                pose = tf_target_buffer_->transform(pose, "Arm_Base");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "Could not transform pose: %s", ex.what());
                return -1;
            }

            return 0;
        }   

        bool execute_cartesian_move(moveit::planning_interface::MoveGroupInterface& move_group, 
                                    const geometry_msgs::msg::Pose& target_pose)
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose); // Linear path from Current -> Target

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0; // Disable jump check for short moves
            const double eef_step = 0.01;      // 1cm resolution

            double fraction = move_group.computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory
            );

            if (fraction < 0.90) {
                // If we can't compute at least 90% of the path, abort
                return false;
            }

            return (move_group.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
        }

        double computeApproachAngle() {
            geometry_msgs::msg::PoseStamped target_pose = geometry_msgs::msg::PoseStamped();
            // Set target pose at the origin of the "target_trash" frame
            target_pose.header.frame_id = "target_trash";  
            target_pose.header.stamp = rclcpp::Time(0);
            target_pose.pose.orientation.w = 1.0;

            // Transform the target pose to the Arm_Base frame using TF2
            try {
                target_pose = tf_target_buffer_->transform(target_pose, "Arm_Base");
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(node_->get_logger(), "Could not transform pose: %s", ex.what());
                return NAN;
            }

            // Switch to polar coordinates
            return atan2(target_pose.pose.position.y, target_pose.pose.position.x);
        }

        geometry_msgs::msg::Quaternion rotateYToOrigin(const geometry_msgs::msg::Point& target_point)
        {
            // Calculate the angle of the vector FROM origin of Shoulder_Rotation_Pitch TO target_point
            double yaw_angle = std::atan2(target_point.x, target_point.z) + (M_PI / 2.0);

            tf2::Quaternion tf_quat;
            tf_quat.setRPY(-M_PI/2.0, -(M_PI / 2.0) + yaw_angle, 0.0);
            
            return tf2::toMsg(tf_quat);
        }

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<std::thread> exec_thread_;
        std::atomic<bool> isRunning_;
        std::atomic<bool> planSuccess_;
        std::atomic<bool> execSuccess_;
        moveit::planning_interface::MoveGroupInterface::Plan finalPlan_;

        // TF2 buffer and listener
        std::unique_ptr<tf2_ros::Buffer> tf_target_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_target_listener_{nullptr};

        // Constants
        const double gripper_asym_offset_ = 0.035;       // Meters
        const double gripper_approach_offset_ = 0.04;   // Meters
};