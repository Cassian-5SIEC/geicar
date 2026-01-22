#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace BT;
class NavToGoal : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {
    public:
        NavToGoal(const std::string& name,
                        const NodeConfig& conf,
                        const RosNodeParams& params)
            : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
        {}
        static PortsList providedPorts() 
        {
            return {InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "The goal pose for navigation")};
        }
        
        bool setGoal(RosActionNode::Goal& goal) override 
        {
            // Get the pose from the blackboard
            Expected<geometry_msgs::msg::PoseStamped> goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
            if (!goal_pose) return false;
            goal.pose = goal_pose.value();
            return true;
        }

        NodeStatus onResultReceived(const WrappedResult& wr) override 
        {
            switch (wr.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(logger(), "Navigation Succeeded!");
                    return BT::NodeStatus::SUCCESS;

                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(logger(), "Navigation Aborted by Nav2");
                    return BT::NodeStatus::FAILURE;

                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(logger(), "Navigation Canceled");
                    return BT::NodeStatus::FAILURE;

                default:
                    return BT::NodeStatus::FAILURE;
            }
        }

        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(logger(), "Error: %d", error);
            return NodeStatus::FAILURE;
        }
};
                    
