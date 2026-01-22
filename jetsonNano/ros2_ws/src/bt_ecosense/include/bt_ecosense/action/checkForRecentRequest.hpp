#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace BT;

class CheckForRecentRequest : public RosTopicSubNode<std_msgs::msg::Bool>
{
  public:
    CheckForRecentRequest(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params)
        : RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override
    {
        // empty if no new message received, since the last tick
        if(last_msg && last_msg->data)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }
};