#include "network_hmi/tcp_udp_bridge_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpUdpBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}