#include "network_hmi/tcp_udp_bridge_node.hpp"
#include "network_hmi/h264_streamer.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nlohmann/json.hpp>
#include <arpa/inet.h> // For htonl, htons
#include <cstring> // For memcpy

using json = nlohmann::json;
using std::placeholders::_1;

TcpUdpBridgeNode::TcpUdpBridgeNode()
: Node("tcp_udp_bridge"),
  image_packet_buffer_(HEADER_SIZE + IMAGE_PACKET_PAYLOAD_SIZE) // Pre-allocate buffer
{
    // Declare and get parameters
    this->declare_parameter<int>("tcp_control_port", 5001);
    this->declare_parameter<int>("udp_data_port", 5000);
    this->declare_parameter<std::string>("image_topic", "/usb_cam_left/image_raw/compressed");
    this->declare_parameter<std::string>("general_data_topic", "/general_data");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("control_topic", "/control_msg");

    tcp_control_port_ = this->get_parameter("tcp_control_port").as_int();
    udp_data_port_ = this->get_parameter("udp_data_port").as_int();
    image_topic_ = this->get_parameter("image_topic").as_string();
    general_data_topic_ = this->get_parameter("general_data_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    control_topic_ = this->get_parameter("control_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting bridge node...");
    RCLCPP_INFO(this->get_logger(), " - TCP Control Port: %d", tcp_control_port_);
    RCLCPP_INFO(this->get_logger(), " - UDP Data Port: %d", udp_data_port_);
    RCLCPP_INFO(this->get_logger(), " - Image Topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), " - General Data Topic: %s", general_data_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), " - Map Topic: %s", map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), " - Control Topic: %s", control_topic_.c_str());

    // --- Create core components ---
    vehicle_state_ = std::make_shared<SharedVehicleState>();
    client_info_ = std::make_shared<SharedClientInfo>(this->get_logger());
    udp_sender_ = std::make_unique<UdpDataSender>();

    // --- TF Init ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // The receiver needs the node to create a publisher
    udp_receiver_ = std::make_unique<UdpDataReceiver>(
        this, 
        vehicle_state_, 
        udp_data_port_,
        "network_joystick_order"
    );
    
    // The server needs the shared state objects to pass to new sessions
    tcp_server_ = std::make_unique<TcpControlServer>(
        this,
        this->get_logger(),
        tcp_control_port_,
        client_info_,
        vehicle_state_,
        control_topic_
    );

    // --- Create ROS subscribers ---
    // Note: joystick_order_pub_ is created inside UdpDataReceiver
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TcpUdpBridgeNode::odom_callback, this, _1)
    );

    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic_, 1, // QoS 1, only need the latest
        std::bind(&TcpUdpBridgeNode::image_callback, this, _1)
    );

    general_data_sub_ = this->create_subscription<interfaces::msg::GeneralData>(
        general_data_topic_, 10,
        std::bind(&TcpUdpBridgeNode::general_data_callback, this, _1)
    );

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, 1,
        std::bind(&TcpUdpBridgeNode::map_callback, this, _1)
    );


    // --- Start network threads ---
    udp_receiver_->start();
    tcp_server_->start();
    
    RCLCPP_INFO(this->get_logger(), "Bridge node started and network threads running.");
}

TcpUdpBridgeNode::~TcpUdpBridgeNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down bridge node...");

    // Stop threads in reverse order with timing logs to diagnose hangs
    {
        auto t0 = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Stopping TcpControlServer...");
        tcp_server_->stop();
        auto t1 = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        RCLCPP_INFO(this->get_logger(), "TcpControlServer stopped (%.1f ms)", static_cast<double>(ms));
    }

    {
        auto t0 = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Stopping UdpDataReceiver...");
        udp_receiver_->stop();
        auto t1 = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        RCLCPP_INFO(this->get_logger(), "UdpDataReceiver stopped (%.1f ms)", static_cast<double>(ms));
    }

    // udp_sender_ and state objects are auto-destroyed
    RCLCPP_INFO(this->get_logger(), "Bridge node shut down complete.");
}

void TcpUdpBridgeNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto dest = client_info_->get_data_address();
    if (!dest.valid) {
        return; // No client connected or client didn't want data
    }

    json real_vel_msg = {
        {"type", "real_vel"},
        {"linear_x", msg->twist.twist.linear.x},
        {"angular_z", msg->twist.twist.angular.z}
    };
    
    udp_sender_->send_json(real_vel_msg.dump(), dest);
}

void TcpUdpBridgeNode::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try {
        // 1. Convert ROS message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 2. Push the image to the streamer
        auto streamer = client_info_->get_h264_streamer();
        if (streamer) {
            streamer->push_image(cv_ptr->image);
        }

    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void TcpUdpBridgeNode::general_data_callback(const interfaces::msg::GeneralData::SharedPtr msg)
{
    static time_t last_log_time = 0;
    time_t current_time = time(nullptr);
    if (current_time - last_log_time < 5)
    { // Ignore logs more frequent than every 5 seconds
        return;    
    }

    auto dest = client_info_->get_data_address();
    if (!dest.valid) {
        return; // No client connected or client didn't want data
    }

    json general_data_msg = {
        {"type", "general_data"},
        {"battery_level", msg->battery_level}
    };
    
    udp_sender_->send_json(general_data_msg.dump(), dest);
    last_log_time = current_time;
}

void TcpUdpBridgeNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    auto dest = client_info_->get_data_address();
    if (!dest.valid) {
        return; // No client connected or client didn't want data
    }

    // 1. Get Car Position from TF
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not transform map to base_link: %s", ex.what());
        return;
    }

    double car_x = transform.transform.translation.x;
    double car_y = transform.transform.translation.y;
    
    // Get car yaw
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, car_yaw;
    m.getRPY(roll, pitch, car_yaw);

    // 2. Define Local Map Parameters
    // We want a fixed size output, e.g., 60x60 pixels
    const int OUTPUT_SIZE = 60; 
    // We want to cover a certain area, e.g., 12m x 12m -> 0.2m/pixel
    const double OUTPUT_RES = 0.1; 
    
    std::vector<int8_t> rotated_map(OUTPUT_SIZE * OUTPUT_SIZE, -1); // Initialize with unknown

    int width = msg->info.width;
    int height = msg->info.height;
    double map_res = msg->info.resolution;
    double map_origin_x = msg->info.origin.position.x;
    double map_origin_y = msg->info.origin.position.y;

    double cos_yaw = cos(car_yaw);
    double sin_yaw = sin(car_yaw);

    // 3. Iterate over output pixels and sample from original map
    for (int y = 0; y < OUTPUT_SIZE; ++y) {
        for (int x = 0; x < OUTPUT_SIZE; ++x) {
            // Convert to local coordinates (center is 0,0)
            // x corresponds to forward (up in image), y to left
            // Image coordinates: u (col) -> right (-y local), v (row) -> down (-x local)?
            // Usually map visualization: x right, y up.
            // Let's verify standard: 
            // - Car frame: x forward, y left.
            // - We want Image: x (col) from left to right, y (row) from top to bottom.
            // - So Top-Center of image should be +x (forward).
            // - Center of image is (OUTPUT_SIZE/2, OUTPUT_SIZE/2).
            
            // Let's map:
            // img_x (0..60) -> local_y (left)
            // img_y (0..60) -> local_x (forward)
            
            // Standard image convention: x is right, y is down.
            // Car frame: x is forward, y is left.
            // We want car forward to be UP in the image.
            // So Image UP (decreasing row/y) = Car X (forward)
            // Image RIGHT (increasing col/x) = Car -Y (right)
            


            // Map image coords to car coords (meters)
            // row y (0 at top) corresponds to +X (forward)
            // Actually, usually:
            // Pixel (x,y): 
            // x (col): right direction. Car frame: -Y (right).
            // y (row): down direction. Car frame: -X (backward).
            // So pixel (0,0) is Front-Left relative to car? No.
            // Let's stick to standard map orientation: North up.
            // Here "Car Up" (Forward) should be Image Up.
            
            // local_x = - (img_cy) * RES (Up is +x)
            // local_y = - (img_cx) * RES (Right is -y)
            
            // Let's refine: 
            // Image center (30,30) is car (0,0).
            // Pixel (30, 0) [Top Middle] -> should be Forward (+X). 
            // dy = 0 - 30 = -30. So -dy * RES = +X. -> X = -(y - 30) * RES
            // Pixel (60, 30) [Right Middle] -> should be Right (-Y).
            // dx = 60 - 30 = +30. So dx * RES = -Y. -> Y = -(x - 30) * RES
            
            double local_x = -(y - OUTPUT_SIZE / 2.0) * OUTPUT_RES;
            double local_y = -(x - OUTPUT_SIZE / 2.0) * OUTPUT_RES;

            // Rotate to Map Frame
            // global_x = car_x + local_x * cos - local_y * sin
            // global_y = car_y + local_x * sin + local_y * cos
            double global_x = car_x + (local_x * cos_yaw - local_y * sin_yaw);
            double global_y = car_y + (local_x * sin_yaw + local_y * cos_yaw);

            // Convert to Map Grid Indices
            int gx = static_cast<int>((global_x - map_origin_x) / map_res);
            int gy = static_cast<int>((global_y - map_origin_y) / map_res);

            // Check bounds
            if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
                rotated_map[y * OUTPUT_SIZE + x] = msg->data[gy * width + gx];
            } else {
                rotated_map[y * OUTPUT_SIZE + x] = -1; // Unknown
            }
        }
    }

    json map_info_msg = {
        {"type", "occupancy_grid"},
        {"width", OUTPUT_SIZE},
        {"height", OUTPUT_SIZE},
        {"resolution", OUTPUT_RES},
        {"origin_position_x", 0.0}, // Local map, origin relative to car? Not quite.
        // Actually, the client probably expects standard map origin logic.
        // But since we are streaming a "live" view centered on the car, the origin changes every frame.
        // If we say origin is (0,0), the client might draw it fixed.
        // But the data is already rotated. 
        // Let's provide 0s for origin and let the client just draw this image as a HUD element or centered map.
        // Or we pass the car position? 
        // For "crop the map around the car", usually it's for a minimap HUD.
        {"origin_position_y", 0.0},
        {"origin_position_z", 0.0},
        {"origin_orientation_x", 0.0},
        {"origin_orientation_y", 0.0},
        {"origin_orientation_z", 0.0},
        {"origin_orientation_w", 1.0},
        {"data", rotated_map}
    };

    udp_sender_->send_json(map_info_msg.dump(), dest);
}