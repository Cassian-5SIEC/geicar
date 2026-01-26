#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BT;

class CheckForModeMsg : public RosTopicSubNode<std_msgs::msg::String>
{
  public:
    CheckForModeMsg(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params)
        : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            OutputPort<std::string>("current_mode", "The current mode"),
        });
    }

    NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override
    {
        // empty if no new message received, since the last tick
        if(last_msg)
        {
            setOutput("current_mode", last_msg->data);
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }
};