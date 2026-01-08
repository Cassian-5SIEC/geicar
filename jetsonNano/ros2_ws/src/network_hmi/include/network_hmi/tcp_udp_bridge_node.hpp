#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/general_data.hpp"
#include "interfaces/msg/control.hpp"

#include "shared_client_info.hpp"
#include "shared_vehicle_state.hpp"
#include "udp_data_sender.hpp"
#include "udp_data_receiver.hpp"
#include "tcp_control_server.hpp"
#include "image_packet_header.hpp"

#include <memory>
#include <vector>

#ifndef MAP_SIZE_LIMIT
#define MAP_SIZE_LIMIT 50 // Pixels in each dimension
#endif

// The main ROS Node class, acts as the "conductor"
class TcpUdpBridgeNode : public rclcpp::Node
{
public:
    TcpUdpBridgeNode();
    ~TcpUdpBridgeNode();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void general_data_callback(const interfaces::msg::GeneralData::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // --- ROS Parameters ---
    int tcp_control_port_;
    int udp_data_port_;
    std::string image_topic_;
    std::string general_data_topic_;
    std::string map_topic_;
    std::string control_topic_;

    // --- ROS Interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Subscription<interfaces::msg::GeneralData>::SharedPtr general_data_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    // The UdpDataReceiver will publish to this, so it's not strictly "owned" by the node
    rclcpp::Publisher<interfaces::msg::JoystickOrder>::SharedPtr joystick_order_pub_;
    rclcpp::Publisher<interfaces::msg::Control>::SharedPtr control_pub_;

    // --- Core Components ---
    std::shared_ptr<SharedVehicleState> vehicle_state_;
    std::shared_ptr<SharedClientInfo> client_info_;
    std::unique_ptr<UdpDataSender> udp_sender_;
    std::unique_ptr<UdpDataReceiver> udp_receiver_;
    std::unique_ptr<TcpControlServer> tcp_server_;

    // --- TF ---
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    // --- Image fragmentation constants ---
    static constexpr int IMAGE_PACKET_PAYLOAD_SIZE = 1400; 
    static constexpr int HEADER_SIZE = sizeof(ImagePacketHeader);
    
    // Reusable buffer for image packets
    std::vector<uint8_t> image_packet_buffer_;
};