#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "interfaces/msg/control.hpp"


using namespace BT;

class SendControlMsg : public RosTopicPubNode<interfaces::msg::Control>
{
  public:
    SendControlMsg(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params)
        : RosTopicPubNode<interfaces::msg::Control>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("command", "Control command to send"),
        });
    }

    bool setMessage(interfaces::msg::Control& msg) override
    {
        Expected<std::string> command = getInput<std::string>("command");
        if (!command) {
            return false;
        }
        msg.sender = "behavior_tree";
        msg.command = command.value();
        return true;
    }
};