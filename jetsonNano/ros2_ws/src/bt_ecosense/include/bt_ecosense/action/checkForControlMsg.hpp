#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "interfaces/msg/control.hpp"

using namespace BT;

class CheckForControlMsg : public RosTopicSubNode<interfaces::msg::Control>
{
  public:
    CheckForControlMsg(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params)
        : RosTopicSubNode<interfaces::msg::Control>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("expected_command", "Expected control command to read"),
        });
    }

    NodeStatus onTick(const std::shared_ptr<interfaces::msg::Control>& last_msg) override
    {
        // empty if no new message received, since the last tick
        if(last_msg)
        {
            Expected<std::string> expected_command = getInput<std::string>("expected_command");
            if(expected_command && last_msg->command == expected_command.value())
            {
                return NodeStatus::SUCCESS;
            }
        }
        return NodeStatus::FAILURE;
    }
};