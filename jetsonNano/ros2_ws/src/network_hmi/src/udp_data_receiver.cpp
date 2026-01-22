#include "network_hmi/udp_data_receiver.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <errno.h>

using json = nlohmann::json;

UdpDataReceiver::UdpDataReceiver(
    rclcpp::Node* node,
    std::shared_ptr<SharedVehicleState> state,
    int port,
    std::string name)
: logger_(node->get_logger().get_child("udp_receiver")),
  vehicle_state_(state),
  data_port_(port),
  name_(name)
{
    // Create the publisher from the node
    joystick_pub_ = node->create_publisher<interfaces::msg::JoystickOrder>(name_, 1);
}

UdpDataReceiver::~UdpDataReceiver()
{
    stop();
}

void UdpDataReceiver::start()
{
    running_ = true;
    thread_ = std::thread(&UdpDataReceiver::receive_loop, this);
    RCLCPP_INFO(logger_, "UDP Server listening on port %d", data_port_);
}

void UdpDataReceiver::stop()
{
    //RCLCPP_INFO(logger_, "UdpDataReceiver::stop() entry");
    running_ = false;
    if (data_socket_ != -1) {
        close(data_socket_); // This unblocks recvfrom()
        data_socket_ = -1;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    //RCLCPP_INFO(logger_, "UdpDataReceiver::stop() exit");
}

void UdpDataReceiver::receive_loop()
{
    struct sockaddr_in servaddr;
    char buffer[1024];

    data_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (data_socket_ < 0) {
        RCLCPP_ERROR(logger_, "UDP socket creation failed");
        return;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(data_port_);

    if (bind(data_socket_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        RCLCPP_ERROR(logger_, "UDP bind failed: %s", strerror(errno));
        return;
    }

    // Set a receive timeout so recvfrom() wakes periodically and we can
    // check `running_` to exit promptly during shutdown.
    struct timeval tv;
    tv.tv_sec = 1; // 1 second timeout
    tv.tv_usec = 0;
    setsockopt(data_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

    while (running_) {
        struct sockaddr_in cliaddr;
        socklen_t len = sizeof(cliaddr);
        // Use no special flags; timeout is handled by SO_RCVTIMEO above.
        ssize_t n = recvfrom(
            data_socket_, (char *)buffer, sizeof(buffer), 0,
            (struct sockaddr *)&cliaddr, &len);
        
        if (n <= 0) {
            // Ignore benign timeout/interrupt errors (they happen regularly
            // due to SO_RCVTIMEO) and only warn on real errors.
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            if (running_) { // Only log if we weren't intentionally stopped
                RCLCPP_WARN(logger_, "UDP recvfrom error: %s", strerror(errno));
            }
            continue;
        }
        buffer[n] = '\0';
        process_packet(buffer, n);
    }
    close(data_socket_);
    data_socket_ = -1;
}

void UdpDataReceiver::process_packet(const char* buffer, ssize_t len)
{
    try {
        json data_msg = json::parse(buffer, buffer + len);
        if (data_msg.value("type", "") == "cmd_vel") {
            float linear_x = data_msg.value("linear_x", 0.0);
            float angular_z = data_msg.value("angular_z", 0.0);

            auto joystick_order = std::make_unique<interfaces::msg::JoystickOrder>();

            if (linear_x < 0.0) {
                joystick_order->throttle = -linear_x; // Ensure throttle is positive
                joystick_order->reverse = true;
            } else {
                joystick_order->throttle = linear_x;
                joystick_order->reverse = false;
            }
            joystick_order->steer = - angular_z;

            // Safely read shared state
            SharedVehicleState::State state = vehicle_state_->get_state();
            joystick_order->start = state.start ? state.mode != 2 : false;
            joystick_order->mode = state.mode;

            joystick_pub_->publish(std::move(joystick_order));
        }
    } catch (json::parse_error & e) {
        RCLCPP_WARN(logger_, "UDP JSON parse error: %s", e.what());
    }
}