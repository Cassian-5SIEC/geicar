#include "network_hmi/h264_streamer.hpp"
#include <stdexcept>

H264Streamer::H264Streamer(rclcpp::Logger logger, 
                           const std::string& host, int port, 
                           int width, int height, int framerate, int bitrate)
    : logger_(logger), width_(width), height_(height), framerate_(framerate), timestamp_(0)
{
    // Initialize GStreamer
    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    // --- PIPELINE CONSTRUCTION ---
    // We replicate your working CLI command almost exactly.
    // CLI: ximagesrc -> videoconvert -> videoscale -> x264enc -> rtph264pay -> udpsink
    // C++: appsrc    -> videoconvert -> videoscale -> x264enc -> rtph264pay -> udpsink
    
    std::string pipeline_str = 
        "appsrc name=ros_src format=3 is-live=true do-timestamp=false ! " 
        "queue max-size-buffers=5 leaky=2 ! "
        "videoconvert ! "
        "videoscale ! video/x-raw,width=1280,height=720 ! " // Ensure output is scaled
        // --- ENCODER CHANGES ---
        "x264enc tune=zerolatency "
        "bitrate=" + std::to_string(bitrate) + " "
        "speed-preset=ultrafast "
        "intra-refresh=true "       // <--- CRITICAL FIX: Self-healing stream
        "key-int-max=60 "           // Refresh cycle size
        "sliced-threads=true ! "    // Lower latency threading
        // -----------------------
        "rtph264pay config-interval=-1 pt=96 mtu=1400 ! " // config-interval=-1 sends headers often
        "udpsink host=" + host + " port=" + std::to_string(port) + " sync=false async=false";

    RCLCPP_INFO(logger_, "Using Pipeline: %s", pipeline_str.c_str());

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error) {
        std::string err_msg = "Pipeline error: " + std::string(error->message);
        g_error_free(error);
        throw std::runtime_error(err_msg);
    }

    // --- APPSRC SETUP ---
    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_src");
    if (!appsrc_) throw std::runtime_error("Could not get appsrc");

    // Tell appsrc what OpenCV is feeding it (BGR)
    // NOTE: We don't need to specify 1280x720 here if the input image is different.
    // 'videoscale' in the pipeline handles the resize to the target width/height.
    // We just need to tell it what the INPUT (OpenCV Mat) format is.
    // Ideally, pass the input image size to this constructor, OR update caps dynamically.
    // For now, assuming constructor width/height matches input image.
    std::string caps_str = "video/x-raw,format=BGR,width=" + std::to_string(width_) + 
                           ",height=" + std::to_string(height_) + 
                           ",framerate=" + std::to_string(framerate_) + "/1";
                           
    GstCaps *caps = gst_caps_from_string(caps_str.c_str());
    gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
    gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);
    gst_caps_unref(caps);

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
}

H264Streamer::~H264Streamer()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
}

void H264Streamer::push_image(const cv::Mat& image)
{
    if (image.empty()) return;

    // 1. Ensure Memory Continuity (Crucial for C++ -> GStreamer)
    cv::Mat frame_to_send;
    if (!image.isContinuous()) {
        frame_to_send = image.clone();
    } else {
        frame_to_send = image;
    }

    // 2. Create Buffer
    gsize data_size = frame_to_send.total() * frame_to_send.elemSize();
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, data_size, NULL);
    gst_buffer_fill(buffer, 0, frame_to_send.data, data_size);

    // 3. Manual Timestamping (The C++ equivalent of ximagesrc's automatic timing)
    GstClockTime duration = gst_util_uint64_scale_int(1, GST_SECOND, framerate_);
    GST_BUFFER_PTS(buffer) = timestamp_;
    GST_BUFFER_DTS(buffer) = timestamp_;
    GST_BUFFER_DURATION(buffer) = duration;
    timestamp_ += duration;

    // 4. Push
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
}