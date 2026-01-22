#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <atomic>
#include <thread> // <-- Make sure <thread> is included
#include <memory>
#include <chrono>
#include <nlohmann/json.hpp>
#include "network_hmi/shared_client_info.hpp"   // <-- Renamed include
#include "network_hmi/shared_vehicle_state.hpp" // <-- Renamed include
#include "network_hmi/h264_streamer.hpp"
#include "interfaces/msg/control.hpp"

// Forward declaration to avoid circular include
class TcpControlServer;

// Manages the complete lifecycle of a single connected TCP client
class ClientSession
{
public:
    ClientSession(
        rclcpp::Logger logger,
        int client_socket,
        std::string client_ip,
        std::shared_ptr<SharedClientInfo> client_info,
        std::shared_ptr<SharedVehicleState> vehicle_state,
        TcpControlServer * tcp_server
    );
    
    // Main function to be run in a new thread
    void run();
    void public_send_tcp_message(const std::string& msg);
    int get_socket() const;

private:
    void handle_message(const nlohmann::json& msg);
    void start_heartbeat_thread();
    bool send_tcp_message(const std::string& msg);
    
    // Message handlers
    void on_register(const nlohmann::json& msg);
    void on_ping();
    void on_emergency_stop();
    void on_close();
    void on_start();
    void on_set_mode(const nlohmann::json& msg);
    void on_heartbeat_ack();

    rclcpp::Logger logger_;
    int socket_;
    std::string ip_;
    std::shared_ptr<SharedClientInfo> client_info_;
    std::shared_ptr<SharedVehicleState> vehicle_state_;
    TcpControlServer * tcp_server_; // <-- ADDED: Pointer to TCP server

    bool registered_this_session_ = false;
    std::atomic<bool> session_alive_{false};
    std::atomic<long long> last_activity_ms_{0};
    std::atomic<long long> last_heartbeat_ms_{0};
    
    std::thread heartbeat_thread_; // <-- ADDED: Store the thread

    // Constants
    static constexpr int TCP_BUFFER_SIZE = 2048;
    static constexpr int CLIENT_ACTIVITY_TIMEOUT_MS = 15000;
    static constexpr int HEARTBEAT_INTERVAL_S = 13;
    static constexpr int CLIENT_READ_TIMEOUT_S = 2;
};