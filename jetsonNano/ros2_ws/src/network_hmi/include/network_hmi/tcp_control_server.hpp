#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include "shared_client_info.hpp"
#include "client_session.hpp"
#include "shared_vehicle_state.hpp"
#include "interfaces/msg/control.hpp"
#include <algorithm>

// Runs in its own thread to accept new TCP connections
class TcpControlServer
{
public:
    TcpControlServer(
        rclcpp::Node* node,
        rclcpp::Logger logger,
        int port,
        std::shared_ptr<SharedClientInfo> client_info,
        std::shared_ptr<SharedVehicleState> vehicle_state,
        std::string control_topic
    );
    ~TcpControlServer();

    void start();
    void stop();
    void send_control_message(const interfaces::msg::Control & msg);
    void remove_client_session(int socket);

private:
    void accept_loop();

    rclcpp::Logger logger_;
    int port_;
    int server_fd_ = -1;
    std::shared_ptr<SharedClientInfo> client_info_;
    std::shared_ptr<SharedVehicleState> vehicle_state_;
    std::string control_topic_;

    rclcpp::Publisher<interfaces::msg::Control>::SharedPtr control_pub_;
    rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_sub_;

    std::thread thread_;
    std::atomic<bool> running_{false};

    std::vector<std::shared_ptr<ClientSession>> client_sessions_;

    std::mutex sessions_mutex_;

    void handle_control_message(const interfaces::msg::Control::SharedPtr msg);
};
