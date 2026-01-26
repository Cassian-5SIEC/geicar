#include "behaviortree_ros2/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

using TrashLocalizationSrv = std_srvs::srv::Trigger;
using namespace BT;

// TO DEFINE THE SERVICE NAME : please set the port "service_name" in the XML file to the desired service name

class CheckTrashInCenter: public RosServiceNode<TrashLocalizationSrv>
{
  public:

  CheckTrashInCenter(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<TrashLocalizationSrv>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
        InputPort<double>("center_threshold", 0.5, "Threshold angle in rad to consider trash in center"),
        OutputPort<std::string>("direction", "Direction of trash in LiDAR frame"),
    });
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override
  {
    // Avoid unused variable warning
    (void)request;
    return true;
  }

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const Response::SharedPtr& response) override
  {
    if (response->success) {
        // Convert response message to angle
        double angle_from_lidar = std::stod(response->message);
        if (angle_from_lidar > M_PI/2 + getInput<double>("center_threshold").value()) {
            setOutput("direction", "left");
            return NodeStatus::FAILURE;
        } else if (angle_from_lidar < M_PI/2 - getInput<double>("center_threshold").value()) {
            setOutput("direction", "right");
            return NodeStatus::FAILURE;
        } else {
            setOutput("direction", "center");
            return NodeStatus::SUCCESS;
        }
    } else {
        RCLCPP_WARN(logger(), "Failed to get angle of trash from Trash Localization: %s", response->message.c_str());
        return NodeStatus::FAILURE;
    }
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }
};