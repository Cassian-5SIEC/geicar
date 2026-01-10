#ifndef H264_STREAMER_HPP_
#define H264_STREAMER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

/**
 * @class H264Streamer
 * @brief A class to encode and stream OpenCV images as H264 over RTP.
 *
 * This class wraps a GStreamer pipeline to handle H264 encoding
 * (using x264enc) and streaming (using udpsink).
 */
class H264Streamer
{
public:
    /**
     * @brief Constructor.
     * @param logger The logger from the parent ROS node.
     * @param host The destination IP address for the stream.
     * @param port The destination port for the stream.
     * @param width The image width.
     * @param height The image height.
     * @param framerate The image framerate.
     * @param bitrate The H264 encoder bitrate in kbit/s.
     */
    H264Streamer(rclcpp::Logger logger, 
                 const std::string& host, int port, 
                 int width, int height, int framerate, int bitrate);

    /**
     * @brief Destructor. Cleans up GStreamer resources.
     */
    ~H264Streamer();

    /**
     * @brief Push a new OpenCV image frame into the GStreamer pipeline.
     * @param image The cv::Mat image to encode and stream. Must be BGR8.
     */
    void push_image(const cv::Mat& image);

private:
    rclcpp::Logger logger_;
    GstElement *pipeline_ = nullptr;
    GstElement *appsrc_ = nullptr;
    int width_, height_, framerate_;
    GstClockTime timestamp_ = 0;
};

#endif // H264_STREAMER_HPP_