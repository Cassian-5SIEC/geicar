#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/control.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CheckRecentRequest : public rclcpp::Node
{
  public:
    CheckRecentRequest()
    : Node("check_recent_request"), recent_request_(false)
    {
        // Declare parameter for expected command
        this->declare_parameter<std::string>("expected_command", "accept-pickup");
        expected_command_ = this->get_parameter("expected_command").as_string();

        // Subscription to the control topic
        control_subscription_ = this->create_subscription<interfaces::msg::Control>(
            "control_msg", 10,
            std::bind(&CheckRecentRequest::topic_callback, this, std::placeholders::_1));

        // Publisher to indicate recent request status
        recent_request_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "recent_request_hmi", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&CheckRecentRequest::timer_callback, this));
    }

  private:
    void topic_callback(const interfaces::msg::Control::SharedPtr msg)
    {
        if (msg->command == expected_command_){
            last_msg_ = msg;
        }
    }

    void timer_callback()
    {
        auto now = this->get_clock()->now();
        recent_request_ = false;
        if (last_msg_ != nullptr && (now - rclcpp::Time(last_msg_->header.stamp) < rclcpp::Duration(500ms))) {
            recent_request_ = true;
            RCLCPP_INFO(this->get_logger(), "Recent request received: %s", last_msg_->command.c_str());
        }
        auto message = std_msgs::msg::Bool();
        message.data = recent_request_;
        recent_request_publisher_->publish(message);
    }

    rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recent_request_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string expected_command_;
    interfaces::msg::Control::SharedPtr last_msg_ = nullptr;
    bool recent_request_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CheckRecentRequest>());
    rclcpp::shutdown();
    return 0;
}