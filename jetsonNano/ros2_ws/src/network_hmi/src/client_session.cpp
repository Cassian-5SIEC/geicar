#include "network_hmi/client_session.hpp" // <-- Renamed include
#include "network_hmi/tcp_control_server.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <cstring> // For strerror

using json = nlohmann::json;

ClientSession::ClientSession(
    rclcpp::Logger logger,
    int client_socket,
    std::string client_ip,
    std::shared_ptr<SharedClientInfo> client_info,
    std::shared_ptr<SharedVehicleState> vehicle_state,
    TcpControlServer * tcp_server)
: logger_(logger),
  socket_(client_socket),
  ip_(client_ip),
  client_info_(client_info),
  vehicle_state_(vehicle_state),
  tcp_server_(tcp_server)
{
    last_activity_ms_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

void ClientSession::run()
{
    RCLCPP_INFO(logger_, "New client session from %s (socket %d)", ip_.c_str(), socket_);

    // Set a read timeout on the socket
    struct timeval tv;
    tv.tv_sec = CLIENT_READ_TIMEOUT_S;
    tv.tv_usec = 0;
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    char buffer[TCP_BUFFER_SIZE];
    
    // Main read loop
    while (rclcpp::ok()) {
        ssize_t valread = read(socket_, buffer, sizeof(buffer));

        if (valread <= 0) {
            if (valread == 0) {
                RCLCPP_INFO(logger_, "Client disconnected gracefully");
                break;
            } else {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // This is our read timeout (SO_RCVTIMEO)
                    long long last_ms = last_activity_ms_.load();
                    long long now = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    
                    if ((now - last_ms) > CLIENT_ACTIVITY_TIMEOUT_MS) {
                        RCLCPP_WARN(
                            logger_, "Client activity timeout (%lld ms), closing session.",
                            now - last_ms);
                        break; // Exit loop, close session
                    }
                    continue; // No activity timeout yet, just a read timeout
                } else if (errno == EINTR) {
                    continue; // Interrupted system call, retry
                } else {
                    RCLCPP_WARN(logger_, "Read error: %s", strerror(errno));
                    break;
                }
            }
        }

        // Valid data received, update last activity time
        last_activity_ms_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

        std::string payload(buffer, static_cast<size_t>(valread));
        // Note: This naive buffering assumes one JSON message per read()
        // A more robust solution would use a delimiter-based buffer.

        try {
            json msg = json::parse(payload);
            handle_message(msg);
        } catch (json::parse_error & e) {
            RCLCPP_WARN(logger_, "JSON parse error: %s", e.what());
            json error_response = {{"ok", false}, {"error", "Invalid JSON"}};
            send_tcp_message(error_response.dump());
        }
    }

    // --- Cleanup ---
    session_alive_.store(false); // Signal heartbeat thread to stop
    close(socket_); // Close socket to unblock send() in heartbeat thread

    // --- MODIFIED ---
    // Wait for the heartbeat thread to finish before we continue
    if (heartbeat_thread_.joinable()) {
        RCLCPP_INFO(logger_, "Waiting for heartbeat thread to join...");
        heartbeat_thread_.join();
        RCLCPP_INFO(logger_, "Heartbeat thread joined.");
    }
    // --- END MODIFIED ---

    if (registered_this_session_) {
        client_info_->deregister_client();
    }
    vehicle_state_->stop_if_not_autonomous();

    RCLCPP_INFO(logger_, "Client session terminated");

    if (tcp_server_ != nullptr) {
        tcp_server_->remove_client_session(socket_);
    }
}

void ClientSession::handle_message(const nlohmann::json& msg)
{
    std::string type = msg.value("type", "");
    
    if (type == "register") on_register(msg);
    else if (type == "ping") on_ping();
    else if (type == "emergency_stop") on_emergency_stop();
    else if (type == "close") on_close();
    else if (type == "start") on_start();
    else if (type == "set_mode") on_set_mode(msg);
    else if (type == "heartbeat_ack") on_heartbeat_ack();
    else {
        RCLCPP_WARN(logger_, "Unknown TCP message type: %s", type.c_str());
        json error_response = {{"ok", false}, {"error", "Unknown message type"}};
        send_tcp_message(error_response.dump());
    }
}

// --- Message Handlers ---

void ClientSession::on_register(const nlohmann::json& msg)
{
    std::string client_id = msg.value("client_id", "client");
    int recv_port = msg.value("recv_udp_port", 0);
    int recv_image_port = msg.value("recv_image_port", 0);
    RCLCPP_INFO(
        logger_, "Register request '%s' (UDP data port: %d, UDP image port: %d)",
        client_id.c_str(), recv_port, recv_image_port);

    if (!client_info_->register_client(ip_, recv_port, recv_image_port)) {
        json response = {{"ok", false}, {"error", "another client already connected"}};
        send_tcp_message(response.dump());
        return;
    }

    registered_this_session_ = true;
    session_alive_.store(true);
    start_heartbeat_thread();

    json response = {
        {"ok", true},
        // TODO: Get this port from the node parameter!
        {"udp_data_port", 5000} 
    };
    send_tcp_message(response.dump());
}

void ClientSession::on_ping()
{
    json response = {{"type", "pong"}};
    send_tcp_message(response.dump());
}

void ClientSession::on_emergency_stop()
{
    RCLCPP_WARN(logger_, "Emergency stop received!");
    vehicle_state_->emergency_stop();
    interfaces::msg::Control control_msg;
    control_msg.command = "stop";
    control_msg.sender = "network_hmi";
    tcp_server_->send_control_message(control_msg);
    json response = {{"ok", true}, {"message", "Emergency stop acknowledged"}};
    send_tcp_message(response.dump());
}

void ClientSession::on_close()
{
    RCLCPP_INFO(logger_, "Client requested close");
    vehicle_state_->stop_if_not_autonomous();
    // This will cause the main read() loop to exit with an error
    shutdown(socket_, SHUT_RDWR); 
}

void ClientSession::on_start()
{
    RCLCPP_INFO(logger_, "Start command received");
    vehicle_state_->set_start(true);
    interfaces::msg::Control control_msg;
    control_msg.command = "start";
    control_msg.sender = "network_hmi";
    tcp_server_->send_control_message(control_msg);
    json response = {{"ok", true}, {"message", "Start command acknowledged"}};
    send_tcp_message(response.dump());
}

void ClientSession::on_set_mode(const nlohmann::json& msg)
{
    int new_mode = msg.value("mode", 0);
    RCLCPP_INFO(logger_, "Set mode received: %d", new_mode);
    vehicle_state_->set_mode(new_mode);
    interfaces::msg::Control control_msg;
    if (new_mode == 0) {
        control_msg.command = "manual";
    } else if (new_mode == 1) {
        control_msg.command = "autonomous";
    } else if (new_mode == 2) {
        control_msg.command = "calibration";
    } else {
        RCLCPP_WARN(logger_, "Unknown mode: %d", new_mode);
        json response = {{"ok", false}, {"error", "Unknown mode"}};
        send_tcp_message(response.dump());
        return;
    }
    control_msg.sender = "network_hmi";
    tcp_server_->send_control_message(control_msg);
    json response = {{"ok", true}, {"message", "Mode change acknowledged"}};
    send_tcp_message(response.dump());
}

void ClientSession::on_heartbeat_ack()
{
    long long now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    long long rtt_ms = now - last_heartbeat_ms_.load();
    RCLCPP_INFO(logger_, "Heartbeat RTT: %lld ms", rtt_ms);
}

// --- Networking Helpers ---

void ClientSession::start_heartbeat_thread()
{
    // --- MODIFIED ---
    // Assign to the member variable and DO NOT detach
    heartbeat_thread_ = std::thread([this]() {
    // --- END MODIFIED ---
        while (session_alive_.load() && rclcpp::ok()) {
            json hb = {{"type", "heartbeat"}};
            
            // Adding this log to match your output
            RCLCPP_INFO(logger_, "Sending heartbeat");

            last_heartbeat_ms_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());

            if (!this->send_tcp_message(hb.dump())) {
                // This will trigger if the socket is closed by the main thread
                RCLCPP_WARN(logger_, "Heartbeat send failed, stopping HB thread.");
                session_alive_.store(false);
                break;
            }
            
            // Sleep for the interval
            for(int i = 0; i < HEARTBEAT_INTERVAL_S && session_alive_.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        RCLCPP_INFO(logger_, "Heartbeat thread stopped.");
    }); // <-- .detach() removed
}

bool ClientSession::send_tcp_message(const std::string &msg)
{
    std::string out = msg + "\n";
    size_t total_sent = 0;
    const char *data = out.data();
    size_t len = out.size();
    while (total_sent < len) {
        ssize_t sent = ::send(socket_, data + total_sent, len - total_sent, 0);
        if (sent < 0) {
            if (errno == EINTR) { continue; } // retry on interrupt
            // Don't spam warnings if the socket was just closed
            if (session_alive_.load()) {
                RCLCPP_WARN(logger_, "TCP send error: %s", strerror(errno));
            }
            return false;
        }
        total_sent += static_cast<size_t>(sent);
    }
    return true;
}

void ClientSession::public_send_tcp_message(const std::string& msg)
{
    send_tcp_message(msg);
}

int ClientSession::get_socket() const
{
    return socket_;
}