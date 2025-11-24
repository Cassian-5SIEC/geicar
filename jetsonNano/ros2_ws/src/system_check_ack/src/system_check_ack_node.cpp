#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include <functional>
#include <memory>
#include <string>


#include "interfaces/msg/system_check.hpp"


using namespace std;
using placeholders::_1;


class system_check_ack : public rclcpp::Node
{

public:
  system_check_ack()
  : Node("system_check_ack_node")
  {

    publisher_system_check_ = this->create_publisher<interfaces::msg::SystemCheck>(
      "system_check",
      10);

    subscription_system_check_ = this->create_subscription<interfaces::msg::SystemCheck>(
      "system_check", 10, std::bind(&system_check_ack::systemCheckCallback, this, _1));

  }

private:
  void systemCheckCallback(const interfaces::msg::SystemCheck & commMsg)
  {

    if (commMsg.request) {
      auto systemCheckMsg = interfaces::msg::SystemCheck();
      systemCheckMsg.response = true;
      systemCheckMsg.jetson = true;
      publisher_system_check_->publish(systemCheckMsg);       //Send communication ACK to system_check_node
    }

  }


  //Publishers
  rclcpp::Publisher<interfaces::msg::SystemCheck>::SharedPtr publisher_system_check_;

  //Subscribers
  rclcpp::Subscription<interfaces::msg::SystemCheck>::SharedPtr subscription_system_check_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<system_check_ack>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
