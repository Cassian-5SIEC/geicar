#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace BT;

class SetVel : public RosTopicPubNode<geometry_msgs::msg::Twist>
{
  public:
    SetVel(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params)
        : RosTopicPubNode<geometry_msgs::msg::Twist>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
      return providedBasicPorts({
        InputPort<double>("linear_x", 0.0, "Linear velocity in X direction"),
        InputPort<double>("angular_z", 0.0, "Angular velocity around Z axis")
      });
    }

    bool setMessage(geometry_msgs::msg::Twist& msg) override
    {
      Expected<double> linear_x = getInput<double>("linear_x");
      if (!linear_x) {
        return false;
      }

      Expected<double> angular_z = getInput<double>("angular_z");
      if (!angular_z) {
        return false;
      }

      msg.linear.x = linear_x.value();
      msg.angular.z = angular_z.value();

      return true;
    }
};

