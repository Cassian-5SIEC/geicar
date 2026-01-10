#include "network_hmi/shared_client_info.hpp"
#include "network_hmi/h264_streamer.hpp"
#include <cstring> // For memset
#include <arpa/inet.h> // For inet_pton

SharedClientInfo::SharedClientInfo(rclcpp::Logger logger)
: logger_(logger)
{}

bool SharedClientInfo::is_connected()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return connected_;
}

bool SharedClientInfo::register_client(const std::string& ip, int data_port, int image_port)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (connected_) {
        return false; // Already have a client
    }

    connected_ = true;
    ip_ = ip;
    data_port_ = data_port;
    image_port_ = image_port;

    // Pre-build the data address struct
    if (data_port_ > 0) {
        memset(&data_addr_.addr, 0, sizeof(data_addr_.addr));
        data_addr_.addr.sin_family = AF_INET;
        data_addr_.addr.sin_port = htons(data_port_);
        inet_pton(AF_INET, ip_.c_str(), &data_addr_.addr.sin_addr);
        data_addr_.valid = true;
    } else {
        data_addr_.valid = false;
    }

    // Pre-build the image address struct
    if (image_port_ > 0) {
        //memset(&image_addr_.addr, 0, sizeof(image_addr_.addr));
        //image_addr_.addr.sin_family = AF_INET;
        //image_addr_.addr.sin_port = htons(image_port_);
        //inet_pton(AF_INET, ip_.c_str(), &image_addr_.addr.sin_addr);
        image_addr_.valid = true;

        // Initialize GStreamer
        h264_streamer_ = std::make_unique<H264Streamer>(logger_, ip_, image_port_, 640, 480, 5, 500);

    } else {
        image_addr_.valid = false;
    }

    return true;
}

void SharedClientInfo::deregister_client()
{
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;
    ip_.clear();
    data_port_ = 0;
    image_port_ = 0;
    data_addr_.valid = false;
    image_addr_.valid = false;
    h264_streamer_.reset();
}

SharedClientInfo::UdpAddress SharedClientInfo::get_data_address()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return data_addr_;
}

SharedClientInfo::UdpAddress SharedClientInfo::get_image_address()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return image_addr_;
}

H264Streamer* SharedClientInfo::get_h264_streamer()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return h264_streamer_.get();
}

