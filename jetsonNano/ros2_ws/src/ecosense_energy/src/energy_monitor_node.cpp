#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/general_data.hpp"
#include "interfaces/msg/energy_status.hpp"

class EnergyMonitorNode : public rclcpp::Node
{
public:
  EnergyMonitorNode()
  : Node("energy_monitor_node")
  {
    general_data_sub_ = this->create_subscription<interfaces::msg::GeneralData>(
      "/general_data", 10,
      std::bind(&EnergyMonitorNode::general_data_callback, this, std::placeholders::_1));

    energy_pub_ = this->create_publisher<interfaces::msg::EnergyStatus>("/energy_status", 10);

    last_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Energy Monitor Node has been started.");
  }

private:
  void general_data_callback(const interfaces::msg::GeneralData::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    
    // Avoid large jumps if messages are sparse or first message
    if (dt > 1.0) {
        dt = 0.0; // Reset dt if gap is too large, or handle as needed
    }
    
    // Integrate power to get energy (Joules = Watts * Seconds)
    total_energy_batt_ += msg->power_batt * dt;
    total_energy_left_ += msg->power_mg * dt;
    total_energy_right_ += msg->power_md * dt;
    total_energy_jet_ += msg->power_jet * dt;

    auto energy_msg = interfaces::msg::EnergyStatus();
    energy_msg.energy_batt = static_cast<float>(total_energy_batt_);
    energy_msg.energy_left = static_cast<float>(total_energy_left_);
    energy_msg.energy_right = static_cast<float>(total_energy_right_);
    energy_msg.energy_jetson = static_cast<float>(total_energy_jet_);

    energy_pub_->publish(energy_msg);

    last_time_ = current_time;
  }

  rclcpp::Subscription<interfaces::msg::GeneralData>::SharedPtr general_data_sub_;
  rclcpp::Publisher<interfaces::msg::EnergyStatus>::SharedPtr energy_pub_;

  rclcpp::Time last_time_;
  double total_energy_batt_ = 0.0;
  double total_energy_left_ = 0.0;
  double total_energy_right_ = 0.0;
  double total_energy_jet_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnergyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
