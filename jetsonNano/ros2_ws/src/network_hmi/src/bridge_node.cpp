#include <rclcpp/rclcpp.hpp>
#include "interfaces/msg/joystick_order.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp> // Requires nlohmann/json library

#include <iostream>
#include <string>
#include <thread>
#include <map>
#include <mutex>
#include <atomic>
#include <chrono>

// For networking
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

using json = nlohmann::json;

class TcpUdpBridge : public rclcpp::Node
{
    private:
    // --- Constants ---
    static constexpr int TCP_BUFFER_SIZE = 2048;
    static constexpr int UDP_BUFFER_SIZE = 1024;
    static constexpr int CLIENT_ACTIVITY_TIMEOUT_MS = 15000;
    static constexpr int HEARTBEAT_INTERVAL_S = 13;
    static constexpr int CLIENT_READ_TIMEOUT_S = 2; // Timeout for socket read()

    public:
    TcpUdpBridge()
    : Node("tcp_udp_bridge")
    {
        // Declare and get ROS 2 parameters
        this->declare_parameter<int>("tcp_control_port", 5001);
        this->declare_parameter<int>("udp_data_port", 5000);

        tcp_control_port_ = this->get_parameter("tcp_control_port").as_int();
        udp_data_port_ = this->get_parameter("udp_data_port").as_int();

        RCLCPP_INFO(this->get_logger(), "Starting the bridge...");
        RCLCPP_INFO(this->get_logger(), "TCP Control Port: %d", tcp_control_port_);
        RCLCPP_INFO(this->get_logger(), "UDP Data Port: %d", udp_data_port_);

        // Create fixed topics (always the same) at startup
        joystick_order_pub_ = this->create_publisher<interfaces::msg::JoystickOrder>(
        "joystick_order", 1);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        // Use a lambda for the callback
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odom_callback(msg);
        }
        );

        // Socket for sending UDP data to clients
        udp_send_socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        // Launch network threads
        tcp_thread_ = std::thread(&TcpUdpBridge::tcp_control_loop, this);
        udp_thread_ = std::thread(&TcpUdpBridge::udp_data_loop, this);
    }

    ~TcpUdpBridge()
    {
        // Proper shutdown
        RCLCPP_INFO(this->get_logger(), "Shutting down bridge...");
        if (tcp_server_fd_ != -1) {
        close(tcp_server_fd_); // Close server socket to unblock accept()
        }
        if (udp_data_socket_ != -1) {
        close(udp_data_socket_); // Close data socket to unblock recvfrom()
        }
        if (udp_send_socket_ != -1) {
        close(udp_send_socket_);
        }

        if (tcp_thread_.joinable()) {
        tcp_thread_.join();
        }
        if (udp_thread_.joinable()) {
        udp_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Bridge shut down complete.");
    }

    private:
    // --- TCP CONTROL THREAD ---
    void tcp_control_loop()
    {
        int new_socket;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);

        tcp_server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_server_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "TCP socket creation failed");
        return;
        }

        setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(tcp_control_port_);

        if (bind(tcp_server_fd_, (struct sockaddr *)&address, sizeof(address)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "TCP bind failed");
        return;
        }

        if (listen(tcp_server_fd_, 3) < 0) {
        RCLCPP_ERROR(this->get_logger(), "TCP listen failed");
        return;
        }

        RCLCPP_INFO(this->get_logger(), "TCP Server listening on port %d", tcp_control_port_);

        while (rclcpp::ok()) {
        new_socket = accept(tcp_server_fd_, (struct sockaddr *)&address, (socklen_t *)&addrlen);
        if (new_socket < 0) {
            // This will happen on shutdown when tcp_server_fd_ is closed
            if (rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "TCP accept error: %s", strerror(errno));
            }
            continue;
        }
        std::string client_ip = inet_ntoa(address.sin_addr);

        // Detach a thread to manage the client session
        std::thread(&TcpUdpBridge::client_session, this, new_socket, client_ip).detach();
        }
        close(tcp_server_fd_);
        tcp_server_fd_ = -1;
    }

    // Helper to send a JSON/text message over a TCP socket with a newline delimiter
    bool send_tcp_message(int sock, const std::string &msg)
    {
        std::string out = msg + "\n";
        size_t total_sent = 0;
        const char *data = out.data();
        size_t len = out.size();
        while (total_sent < len) {
        ssize_t sent = ::send(sock, data + total_sent, len - total_sent, 0);
        if (sent < 0) {
            if (errno == EINTR) {continue;} // retry on interrupt
            RCLCPP_WARN(this->get_logger(), "TCP send error: %s", strerror(errno));
            return false;
        }
        total_sent += static_cast<size_t>(sent);
        }
        return true;
    }

    // --- Manage a TCP client session ---
    void client_session(int client_socket, std::string client_ip)
    {
        char buffer[TCP_BUFFER_SIZE];
        RCLCPP_INFO(
        this->get_logger(), "New client session from %s (socket %d)",
        client_ip.c_str(), client_socket);

        // Set a read timeout on the socket
        struct timeval tv;
        tv.tv_sec = CLIENT_READ_TIMEOUT_S;
        tv.tv_usec = 0;
        setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);


        bool registered_in_this_session = false;
        auto session_alive = std::make_shared<std::atomic<bool>>(false);
        auto last_activity_ms = std::make_shared<std::atomic<long long>>(0);

        auto now_ms = []() -> long long {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        };
        
        // Mark activity at connection start
        last_activity_ms->store(now_ms());

        while (rclcpp::ok()) {
        ssize_t valread = read(client_socket, buffer, sizeof(buffer));

        if (valread <= 0) {
            if (valread == 0) {
            // Client disconnected gracefully
            RCLCPP_INFO(this->get_logger(), "Client %s disconnected (socket %d)", client_ip.c_str(), client_socket);
            break;
            } else {
            // read() error
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // This is our read timeout (SO_RCVTIMEO)
                // Now we check for application-level activity timeout
                long long last_ms = last_activity_ms->load();
                long long now = now_ms();
                if ((now - last_ms) > CLIENT_ACTIVITY_TIMEOUT_MS) {
                RCLCPP_WARN(
                    this->get_logger(), "Client activity timeout (%lld ms) for %s, closing session.",
                    now - last_ms, client_ip.c_str());
                break; // Exit loop, close session
                }
                // No activity timeout yet, just a read timeout. Continue loop.
                continue;
            } else if (errno == EINTR) {
                // Interrupted system call, retry
                continue;
            } else {
                // Other read error
                RCLCPP_WARN(
                this->get_logger(), "Read error on socket %d: %s", client_socket, strerror(errno));
                break;
            }
            }
        }

        // Valid data received, update last activity time
        last_activity_ms->store(now_ms());

        std::string payload(buffer, static_cast<size_t>(valread));

        try {
            json msg = json::parse(payload);

            if (msg.contains("type") && msg["type"] == "register") {
            std::string client_id = msg.value("client_id", "client");
            int recv_port = msg.value("recv_udp_port", 0);
            RCLCPP_INFO(
                this->get_logger(), "Register request '%s' from %s (UDP return port: %d)",
                client_id.c_str(), client_ip.c_str(), recv_port);

            // Register for the single-client model
            {
                std::lock_guard<std::mutex> lock(single_client_mutex_);
                if (single_client_connected_) {
                // A client is already connected
                json response = {{"ok", false}, {"error", "another client already connected"}};
                send_tcp_message(client_socket, response.dump());
                continue;
                }
                single_client_connected_ = true;
                single_client_ip_ = client_ip;
                single_client_recv_port_ = recv_port;
            }
            registered_in_this_session = true;

            // Mark session as alive and start heartbeat
            session_alive->store(true);
            last_activity_ms->store(now_ms());
            std::weak_ptr<std::atomic<bool>> weak_alive = session_alive;
            int hb_socket = client_socket; // capture copy

            // This thread *only* sends heartbeats. Timeout is checked in this (client_session) thread.
            std::thread(
                [this, weak_alive, hb_socket]() {
                if (weak_alive.expired()) {return;}
                auto alive = weak_alive.lock();
                while (alive && alive->load() && rclcpp::ok()) {
                    json hb = {{"type", "heartbeat"}};
                    
                    // Update the last sent heartbeat time
                    last_heartbeat_ms_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());

                    if (!this->send_tcp_message(hb_socket, hb.dump())) {
                        // Failed to send, session is likely dead.
                        RCLCPP_WARN(this->get_logger(), "Heartbeat send failed, stopping HB thread.");
                        alive->store(false);
                        break;
                    }
                    
                    // Wait for heartbeat interval
                    std::this_thread::sleep_for(std::chrono::seconds(HEARTBEAT_INTERVAL_S));
                }
                }).detach();

            // Respond to client (keep connection open)
            json response = {
                {"ok", true},
                {"udp_data_port", udp_data_port_}
            };
            send_tcp_message(client_socket, response.dump());

            } else if (msg.contains("type") && msg["type"] == "ping") {
                // Respond to ping
                json response = {{"type", "pong"}};
                send_tcp_message(client_socket, response.dump());
            } else if (msg.contains("type") && msg["type"] == "emergency_stop") {
                RCLCPP_WARN(this->get_logger(), "Emergency stop received from %s", client_ip.c_str());
                
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    start_ = false;
                }

                json response = {{"ok", true}, {"message", "Emergency stop acknowledged"}};
                send_tcp_message(client_socket, response.dump());
            } else if (msg.contains("type") && msg["type"] == "close") {
                // Client requests to close session

                {   // On session end, ensure vehicle is stopped if not in autonomous mode
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    if (mode_ != 1) { // Do not stop if in autonomous mode
                        start_ = false;
                    }
                }

                break;
            } else if (msg.contains("type") && msg["type"] == "start") {
            RCLCPP_INFO(this->get_logger(), "Start command received from %s", client_ip.c_str());

            {  
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (mode_ != 2) {
                start_ = true;
                }
            }

            json response = {{"ok", true}, {"message", "Start command acknowledged"}};
            send_tcp_message(client_socket, response.dump());
            } else if (msg.contains("type") && msg["type"] == "set_mode") {
            int new_mode = msg.value("mode", 0);
            RCLCPP_INFO(
                this->get_logger(), "Set mode received from %s : %d",
                client_ip.c_str(), new_mode);

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                mode_ = new_mode;
                if (mode_ == 2) {
                start_ = false;
                }
            }

            json response = {{"ok", true}, {"message", "Mode change acknowledged"}};
            send_tcp_message(client_socket, response.dump());
            } else if (msg.contains("type") && msg["type"] == "heartbeat_ack") {
            RCLCPP_INFO(
                this->get_logger(), "Heartbeat ack from %s",
                client_ip.c_str());

            // Calculate RTT
            long long now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            long long rtt_ms = now - last_heartbeat_ms_.load();
            RCLCPP_INFO(
                this->get_logger(), "Heartbeat RTT with %s : %lld ms",
                client_ip.c_str(), rtt_ms);
            } else {
            // Unhandled messages
            RCLCPP_WARN(
                this->get_logger(), "Unknown TCP message from %s: %s",
                client_ip.c_str(), payload.c_str());
            json error_response = {{"ok", false}, {"error", "Unknown message type"}};
            send_tcp_message(client_socket, error_response.dump());
            }

        } catch (json::parse_error & e) {
            RCLCPP_WARN(
            this->get_logger(), "JSON parse error from %s: %s",
            client_ip.c_str(), e.what());
            json error_response = {{"ok", false}, {"error", "Invalid JSON"}};
            send_tcp_message(client_socket, error_response.dump());
        }
        }

        // Close socket when session ends
        session_alive->store(false); // Signal heartbeat thread to stop
        close(client_socket);

        // If this session had registered the client, disconnect it
        if (registered_in_this_session) {
        std::lock_guard<std::mutex> lock(single_client_mutex_);
        single_client_connected_ = false;
        single_client_ip_.clear();
        single_client_recv_port_ = 0;
        }

        {   // On session end, ensure vehicle is stopped if not in autonomous mode
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (mode_ != 1) { // Do not stop if in autonomous mode
                start_ = false;
            }
        }

        RCLCPP_INFO(
        this->get_logger(), "Client session %s (socket %d) terminated",
        client_ip.c_str(), client_socket);
    }

    // --- UDP DATA THREAD (Receiving) ---
    void udp_data_loop()
    {
        struct sockaddr_in servaddr, cliaddr;
        char buffer[UDP_BUFFER_SIZE];

        udp_data_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_data_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "UDP socket creation failed");
        return;
        }

        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(udp_data_port_);

        if (bind(udp_data_socket_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "UDP bind failed");
        return;
        }

        RCLCPP_INFO(this->get_logger(), "UDP Server listening on port %d", udp_data_port_);

        while (rclcpp::ok()) {
        socklen_t len = sizeof(cliaddr);
        ssize_t n = recvfrom(
            udp_data_socket_, (char *)buffer, UDP_BUFFER_SIZE, MSG_WAITALL,
            (struct sockaddr *)&cliaddr, &len);
        if (n <= 0) {
            if (rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "UDP recvfrom error: %s", strerror(errno));
            }
            continue;
        }
        buffer[n] = '\0';

        try {
            json data_msg = json::parse(buffer);
            if (data_msg.value("type", "") == "cmd_vel") {
            // In single-client model, publish directly to the global topic
            float linear_x = data_msg.value("linear_x", 0.0);
            float angular_z = data_msg.value("angular_z", 0.0);

            auto joystick_order = std::make_shared<interfaces::msg::JoystickOrder>();

            if (linear_x < 0.0) {
                linear_x = -linear_x; // Ensure speed is positive
                joystick_order->reverse = true;
            } else {
                joystick_order->reverse = false;
            }

            joystick_order->throttle = linear_x;
            joystick_order->steer = angular_z;

            // Safely read shared state
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                joystick_order->start = start_ ? mode_ != 2 : false;
                joystick_order->mode = mode_;
            }


            if (joystick_order_pub_) {
                joystick_order_pub_->publish(*joystick_order);
            }
            }
        } catch (json::parse_error & e) {
            RCLCPP_WARN(this->get_logger(), "UDP JSON parse error: %s", e.what());
        }
        }
        close(udp_data_socket_);
        udp_data_socket_ = -1;
    }

    // --- Odom Callback (sends data via UDP) ---
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::string client_ip;
        int client_port = 0;

        // Safely get client info
        {
        std::lock_guard<std::mutex> lock(single_client_mutex_);
        if (!single_client_connected_) {
            return; // No client connected
        }
        client_ip = single_client_ip_;
        client_port = single_client_recv_port_;
        }

        // Format the real velocity message
        json real_vel_msg = {
        {"type", "real_vel"},
        {"linear_x", msg->twist.twist.linear.x},
        {"angular_z", msg->twist.twist.angular.z}
        };
        std::string payload = real_vel_msg.dump();

        // Prepare destination address
        struct sockaddr_in dest_addr;
        memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(client_port);
        inet_pton(AF_INET, client_ip.c_str(), &dest_addr.sin_addr);

        // Send UDP packet
        sendto(
        udp_send_socket_, payload.c_str(), payload.length(), 0,
        (const struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }

    // === Class Members ===

    // --- Single-Client Model ---
    bool single_client_connected_ = false;
    std::string single_client_ip_;
    int single_client_recv_port_ = 0;
    std::mutex single_client_mutex_;

    // --- Shared State (Thread-Safe) ---
    int mode_ = 0;
    bool start_ = false;
    std::mutex state_mutex_; // Protects mode_ and start_
    std::atomic<long long> last_heartbeat_ms_{0};

    // --- ROS Topics ---
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr joystick_order_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // --- Network Parameters ---
    int tcp_control_port_;
    int udp_data_port_;

    // --- Sockets ---
    int tcp_server_fd_ = -1;   // TCP listen socket
    int udp_data_socket_ = -1; // Socket for receiving cmd_vel
    int udp_send_socket_ = -1; // Socket for sending real_vel

    // --- Threads ---
    std::thread tcp_thread_;
    std::thread udp_thread_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpUdpBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}