#pragma once

#include <rclcpp/rclcpp.hpp>
#include "interfaces/msg/joystick_order.hpp"
#include "shared_vehicle_state.hpp"
#include <thread>
#include <atomic>
#include <nlohmann/json.hpp>

// Runs in its own thread to receive UDP data (cmd_vel)
class UdpDataReceiver
{
public:
    UdpDataReceiver(
        rclcpp::Node* node, // For publishing
        std::shared_ptr<SharedVehicleState> state,
        int port,
        std::string name
    );
    ~UdpDataReceiver();

    void start();
    void stop();

private:
    void receive_loop();
    void process_packet(const char* buffer, ssize_t len);

    rclcpp::Logger logger_;
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr joystick_pub_;
    std::shared_ptr<SharedVehicleState> vehicle_state_;
    
    int data_port_;
    std::string name_;
    int data_socket_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};

    int mode_;

};