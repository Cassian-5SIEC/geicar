#include "network_hmi/tcp_control_server.hpp"
#include "network_hmi/client_session.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <errno.h>

TcpControlServer::TcpControlServer(
    rclcpp::Node* node,
    rclcpp::Logger logger,
    int port,
    std::shared_ptr<SharedClientInfo> client_info,
    std::shared_ptr<SharedVehicleState> vehicle_state,
    std::string control_topic)
: logger_(logger.get_child("tcp_server")),
  port_(port),
  client_info_(client_info),
  vehicle_state_(vehicle_state),
  control_topic_(control_topic)
{
    control_pub_ = node->create_publisher<interfaces::msg::Control>(control_topic_, 1);
    control_sub_ = node->create_subscription<interfaces::msg::Control>(control_topic_, 1, std::bind(&TcpControlServer::handle_control_message, this, std::placeholders::_1));
}

TcpControlServer::~TcpControlServer()
{
    try {
        stop();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Exception in ~TcpControlServer: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(logger_, "Unknown exception in ~TcpControlServer");
    }
}

void TcpControlServer::start()
{
    running_ = true;
    thread_ = std::thread(&TcpControlServer::accept_loop, this);
}

void TcpControlServer::stop()
{
    //RCLCPP_INFO(logger_, "TcpControlServer::stop() entry");
    running_ = false;

    // Close the listening socket first to unblock accept()
    if (server_fd_ != -1) {
        // First try shutdown to interrupt accept/listen
        shutdown(server_fd_, SHUT_RDWR);
        close(server_fd_); // This should unblock accept()
        server_fd_ = -1;
    }

    // Make a local copy of client sockets while holding the mutex,
    // then release the lock and shutdown/close each socket to
    // promptly unblock client session threads.
    std::vector<int> client_sockets;
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (auto &s : client_sessions_) {
            if (s) client_sockets.push_back(s->get_socket());
        }
    }

    for (int sock : client_sockets) {
        if (sock != -1) {
            shutdown(sock, SHUT_RDWR);
            close(sock);
        }
    }

    if (thread_.joinable()) {
        thread_.join();
    }
    //RCLCPP_INFO(logger_, "TcpControlServer::stop() exit");
}

void TcpControlServer::accept_loop()
{
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        RCLCPP_ERROR(logger_, "TCP socket creation failed");
        return;
    }

    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr *)&address, sizeof(address)) < 0) {
        RCLCPP_ERROR(logger_, "TCP bind failed: %s", strerror(errno));
        return;
    }

    if (listen(server_fd_, 3) < 0) {
        RCLCPP_ERROR(logger_, "TCP listen failed");
        return;
    }

    RCLCPP_INFO(logger_, "TCP Server listening on port %d", port_);

    while (running_) {
        RCLCPP_DEBUG(logger_, "accept_loop: waiting for new connection (server_fd=%d)", server_fd_);
        int new_socket = accept(server_fd_, (struct sockaddr *)&address, (socklen_t *)&addrlen);
        if (new_socket < 0) {
            int err = errno;
            if (running_) {
                RCLCPP_WARN(logger_, "TCP accept error: %s", strerror(err));
            } else {
                RCLCPP_DEBUG(logger_, "accept returned error %s after running_ set false", strerror(err));
            }
            continue;
        }
        std::string client_ip = inet_ntoa(address.sin_addr);

        // Create a new session object and run it in a detached thread
        auto session = std::make_shared<ClientSession>(
            logger_.get_child("session_" + std::to_string(new_socket)),
            new_socket,
            client_ip,
            client_info_,
            vehicle_state_,
            this
        );
        {
            std::lock_guard<std::mutex> lock(sessions_mutex_);
            client_sessions_.push_back(session);
        }
        std::thread(&ClientSession::run, session).detach();
    }
    close(server_fd_);
    server_fd_ = -1;
}

void TcpControlServer::remove_client_session(int socket)
{
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    auto it = std::remove_if(
        client_sessions_.begin(),
        client_sessions_.end(),
        [socket](const std::shared_ptr<ClientSession>& session) {
            return session->get_socket() == socket;
        });
    if (it != client_sessions_.end()) {
        client_sessions_.erase(it, client_sessions_.end());
        RCLCPP_INFO(logger_, "Removed client session for socket %d", socket);
    }
}

void TcpControlServer::send_control_message(const interfaces::msg::Control & msg)
{
    control_pub_->publish(msg);
}

void TcpControlServer::handle_control_message(const interfaces::msg::Control::SharedPtr msg)
{
    // Only forward messages not sent by network_hmi itself
    if (msg->sender != "network_hmi") {
        // copy pointers while holding lock
        std::vector<std::shared_ptr<ClientSession>> sessions_copy;
        {
            std::lock_guard<std::mutex> lock(sessions_mutex_);
            sessions_copy = client_sessions_;
        }

        for (auto& session : sessions_copy) {
            nlohmann::json json_msg = {
                // TODO: fill in message
            };
            
            if (msg->command == "start") {
                json_msg["type"] = "cmd";
                json_msg["cmd"] = "start";
            } else if (msg->command == "stop") {
                json_msg["type"] = "cmd";
                json_msg["cmd"] = "stop";
            } else if (msg->command == "manual") {
                json_msg["type"] = "cmd";
                json_msg["cmd"] = "set_mode";
                json_msg["mode"] = 0;
            } else if (msg->command == "autonomous") {
                json_msg["type"] = "cmd";
                json_msg["cmd"] = "set_mode";
                json_msg["mode"] = 1;
            } else if (msg->command == "calibration") {
                json_msg["type"] = "cmd";
                json_msg["cmd"] = "set_mode";
                json_msg["mode"] = 2;
            } else {
                continue; // Unknown command
            }

            session->public_send_tcp_message(json_msg.dump());
        }
    }
}