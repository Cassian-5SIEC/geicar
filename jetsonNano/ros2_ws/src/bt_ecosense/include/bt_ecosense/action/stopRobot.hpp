#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "interfaces/msg/control.hpp"


using namespace BT;

class StopRobot : public RosTopicPubNode<interfaces::msg::Control>
{
  public:
    StopRobot(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params)
        : RosTopicPubNode<interfaces::msg::Control>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
      return providedBasicPorts({});
    }

    bool setMessage(interfaces::msg::Control& msg) override
    {
        msg.sender = "behavior_tree";
        msg.command = "stop";
        return true;
    }
};