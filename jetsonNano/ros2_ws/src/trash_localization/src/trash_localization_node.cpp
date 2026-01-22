#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"


#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

#define USE_INTERSECTION true

class TrashLocalizationNode : public rclcpp::Node 
{   
    public:
        TrashLocalizationNode() : Node("trash_localization_node"){
            // Declare parameters with default values
            this->declare_parameter<std::double_t>("update_period_in_s", 0.5);
            this->declare_parameter<std::string>("camera_left_target_topic", "/usb_cam_left/object_target");
            this->declare_parameter<std::string>("camera_right_target_topic", "/usb_cam_right/object_target");
            this->declare_parameter<std::string>("camera_left_info_topic", "/usb_cam_left/camera_info");
            this->declare_parameter<std::string>("camera_right_info_topic", "/usb_cam_right/camera_info");
            this->declare_parameter<std::string>("lidar_scan_topic", "/ld_lidar/scan");
            this->declare_parameter<std::string>("left_camera_frame", "camera_left_link");
            this->declare_parameter<std::string>("right_camera_frame", "camera_right_link");
            this->declare_parameter<std::string>("lidar_frame", "ld_lidar_link");
            this->declare_parameter<std::double_t>("tf_timeout", 3.0);
            this->declare_parameter<std::double_t>("left_angle_offset_deg", -1.0);
            this->declare_parameter<std::double_t>("right_angle_offset_deg",5.0);
            this->declare_parameter<bool>("auto_publish", false);

            // Get parameters
            auto update_period_ = std::chrono::duration<double>(this->get_parameter("update_period_in_s").as_double()); 
            tf_timeout_ = this->get_parameter("tf_timeout").as_double();
            left_camera_frame_ = this->get_parameter("left_camera_frame").as_string();
            right_camera_frame_ = this->get_parameter("right_camera_frame").as_string();
            lidar_frame_ = this->get_parameter("lidar_frame").as_string();
            left_angle_offset_deg_ = this->get_parameter("left_angle_offset_deg").as_double();
            right_angle_offset_deg_ = this->get_parameter("right_angle_offset_deg").as_double();
            auto_publish_ = this->get_parameter("auto_publish").as_bool();

            // Subscribers from both cameras
            rclcpp::SensorDataQoS qos;
            qos.keep_last(1);

            camera_left_target_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2D>(
                this->get_parameter("camera_left_target_topic").as_string(), 
                qos,
                std::bind(&TrashLocalizationNode::camera_target_callback, this, std::placeholders::_1)
            );

            camera_left_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                this->get_parameter("camera_left_info_topic").as_string(), 
                qos,
                std::bind(&TrashLocalizationNode::camera_info_callback, this, std::placeholders::_1)
            );

            camera_right_target_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2D>(
                this->get_parameter("camera_right_target_topic").as_string(), 
                qos,
                std::bind(&TrashLocalizationNode::camera_target_callback, this, std::placeholders::_1)
            );
            camera_right_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                this->get_parameter("camera_right_info_topic").as_string(), 
                qos,
                std::bind(&TrashLocalizationNode::camera_info_callback, this, std::placeholders::_1)
            );

            // Subscriber for LIDAR scan
            lidar_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                this->get_parameter("lidar_scan_topic").as_string(),
                rclcpp::SensorDataQoS(),
                std::bind(&TrashLocalizationNode::lidar_scan_callback, this, std::placeholders::_1)
            );

            // TF2 listeners for both cameras
            tf_cam_left_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_left_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_cam_left_buffer_);
            tf_cam_right_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_right_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_cam_right_buffer_);

            // TF2 broadcaster
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // Service to publish target TF when called
            publish_target_tf_service_ = this->create_service<std_srvs::srv::Trigger>(
                "trash_localization_node/publish_target_tf",
                std::bind(&TrashLocalizationNode::process_data, this, std::placeholders::_1, std::placeholders::_2)
            );

            clear_target_tf_service_ = this->create_service<std_srvs::srv::Trigger>(
                "trash_localization_node/clear_target_tf",
                std::bind(&TrashLocalizationNode::clear_target_tf, this, std::placeholders::_1, std::placeholders::_2)
            );

            // Service to get the latest computed target angle (radians)
            get_target_angle_service_ = this->create_service<std_srvs::srv::Trigger>(
                "trash_localization_node/get_target_angle",
                std::bind(&TrashLocalizationNode::get_target_angle, this, std::placeholders::_1, std::placeholders::_2)
            );

            // Publisher for search zone markers
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("trash_localization_node/search_zone", 10);

            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Trash Localization Node has been started.");
            if (auto_publish_) {
                RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Auto-publishing is enabled. Computing every %f seconds.", update_period_.count());
                timer_ = this->create_wall_timer(
                    update_period_, 
                    std::bind(&TrashLocalizationNode::update_target_tf, this)
                );
            }
            
        }

    private:
        void camera_target_callback(const vision_msgs::msg::Detection2D::SharedPtr msg) {
            if (msg->header.frame_id == left_camera_frame_) {
                left_camera_target_ = *msg;
                // RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Received left target stamped at %f seconds.", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
            } else if (msg->header.frame_id == right_camera_frame_) {
                right_camera_target_ = *msg;
                // RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Received right target stamped at %f seconds.", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
            } else {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Received target from unknown camera frame: %s", msg->header.frame_id.c_str());
            }
        }

        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            if (msg->header.frame_id == left_camera_frame_) {
                left_camera_info_ = *msg;
            } else if (msg->header.frame_id == right_camera_frame_) {
                right_camera_info_ = *msg;
            } else {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Received camera info from unknown camera frame: %s", msg->header.frame_id.c_str());
            }
        }

        void lidar_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            latest_lidar_scan_ = *msg;
        }

        geometry_msgs::msg::TransformStamped get_camera_transform(const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time_point) {
            geometry_msgs::msg::TransformStamped tf;
            try {
                if (source_frame == left_camera_frame_) {
                    tf = tf_cam_left_buffer_->lookupTransform(target_frame, source_frame, time_point);
                } else {
                    tf = tf_cam_right_buffer_->lookupTransform(target_frame, source_frame, time_point);
                }
            } catch (tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
                // Fallback to latest if specific time fails (optional, but good for robustness if strict sync fails)
                try {
                     if (source_frame == left_camera_frame_) {
                        tf = tf_cam_left_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                    } else {
                        tf = tf_cam_right_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                    }
                } catch (...) {}
            }
            return tf;
        }

        /**
         * @brief Perform lookup in scan data to find the target detected by the camera, and broadcast its TF.
         * @note Currently only work with left camera data, can be extended to right camera similarly.
         * @note Broadcast the closest valid point in LIDAR scan as target position.
         * 
         */
        bool perform_localization(std::string & message) {
            // RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Processing data to localize target...");
            // Perform localization using both cameras if available
            bool left_valid = false;
            bool right_valid = false;
            double now_s = this->now().seconds();

            if (left_camera_target_.header.stamp.sec != 0) {
                 double left_time = rclcpp::Time(left_camera_target_.header.stamp).seconds();
                 if ((now_s - left_time) < tf_timeout_) {
                     left_valid = true;
                 }
            }
            if (right_camera_target_.header.stamp.sec != 0) {
                 double right_time = rclcpp::Time(right_camera_target_.header.stamp).seconds();
                 if ((now_s - right_time) < tf_timeout_) {
                     right_valid = true;
                 }
            }

            double target_angle = 0.0;
            geometry_msgs::msg::PoseStamped target_pose_intersect;
            bool use_intersection = false;

            // Dual Camera Angle Averaging (Sensor Fusion)
            if (left_valid && right_valid) {
                #if USE_INTERSECTION
                
                double angle_left_cam = compute_angle_from_camera(left_camera_target_, left_camera_info_, left_camera_frame_);
                double angle_right_cam = compute_angle_from_camera(right_camera_target_, right_camera_info_, right_camera_frame_);
                
                // transform_angle_to_lidar_frame with proper timestamps
                rclcpp::Time left_time = left_camera_target_.header.stamp;
                rclcpp::Time right_time = right_camera_target_.header.stamp;
                
                double angle_left_lidar = transform_angle_to_lidar_frame(angle_left_cam, left_camera_frame_, lidar_frame_, left_time);
                double angle_right_lidar = transform_angle_to_lidar_frame(angle_right_cam, right_camera_frame_, lidar_frame_, right_time);

                // Get transforms at specific times
                auto tf_left = get_camera_transform(lidar_frame_, left_camera_frame_, tf2_ros::fromMsg(left_camera_target_.header.stamp));
                auto tf_right = get_camera_transform(lidar_frame_, right_camera_frame_, tf2_ros::fromMsg(right_camera_target_.header.stamp));

                geometry_msgs::msg::PoseStamped intersection_pose = compute_intersection(angle_left_lidar, angle_right_lidar, tf_left, tf_right);
                
                if (!std::isnan(intersection_pose.pose.position.x)) {
                    target_angle = atan2(intersection_pose.pose.position.y, intersection_pose.pose.position.x);
                    use_intersection = true;
                }

                #else
                //RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Both cameras detecting target. Averaging angles...");
                #endif
            }

            if (!use_intersection) {
                if (right_valid) {
                    double angle_cam = compute_angle_from_camera(right_camera_target_, right_camera_info_, right_camera_frame_);
                    auto target_pose = compute_pose_from_camera_angle(right_camera_target_, right_camera_info_, angle_cam, right_camera_frame_);
                    target_angle = compute_lidar_angle_from_camera_pose(target_pose, right_camera_frame_);
                    RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Target angle: %f", target_angle);
                    //target_angle = transform_angle_to_lidar_frame(angle_cam, right_camera_frame_, lidar_frame_, right_camera_target_.header.stamp);
                } else if (left_valid) {
                    double angle_cam = compute_angle_from_camera(left_camera_target_, left_camera_info_, left_camera_frame_);
                    auto target_pose = compute_pose_from_camera_angle(left_camera_target_, left_camera_info_, angle_cam, left_camera_frame_);
                    target_angle = compute_lidar_angle_from_camera_pose(target_pose, left_camera_frame_);
                    RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Target angle: %f", target_angle);
                    //target_angle = transform_angle_to_lidar_frame(angle_cam, left_camera_frame_, lidar_frame_, left_camera_target_.header.stamp);
                } else {
                     // Calculate delays for logging
                     double left_diff = (left_camera_target_.header.stamp.sec != 0) ? (now_s - rclcpp::Time(left_camera_target_.header.stamp).seconds()) : -1.0;
                     double right_diff = (right_camera_target_.header.stamp.sec != 0) ? (now_s - rclcpp::Time(right_camera_target_.header.stamp).seconds()) : -1.0;

                     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "No valid camera data (Age: L=%.1fs, R=%.1fs). If playing bag, check 'use_sim_time' and '--clock'.",
                        left_diff, right_diff);

                     message = "No valid camera data.";
                     return false;
                }
            }
            
            if (std::isnan(target_angle)) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Invalid target angle.");
                message = "Invalid target angle.";
                return false;
            }

            // Normalize angle to (-PI, PI]
            target_angle = atan2(sin(target_angle), cos(target_angle));

            // Safety Check: Trash must be in front relative to LIDAR frame (0 to 180 degrees)
            // If the angle is backwards, it's likely a math glitch or parallel ray ghost intersection.
            if (target_angle > M_PI || target_angle < 0) {
                 RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Calculated angle %.2f rad is behind the robot! Ignoring.", target_angle);
                 message = "Invalid target angle (behind robot).";
                 return false;
            }

            // Always use angular search (Cone) based on the best estimated angle.
            // This avoids "too close" distance issues from poor triangulation depth.
            // store latest estimated angle for external queries
            latest_target_angle_ = target_angle;
            int target_index = find_target_in_lidar_scan(target_angle, latest_lidar_scan_);

            if (target_index >= 0) {
                broadcast_target_tf(latest_lidar_scan_.angle_min + target_index * latest_lidar_scan_.angle_increment, latest_lidar_scan_.ranges[target_index], lidar_frame_);
                message = "Target localized and TF broadcasted.";
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Target not found in LIDAR scan.");
                message = "Target not found in LIDAR scan.";
                return false;
            }
        }

        void update_target_tf() {
            std::string message;
            perform_localization(message);
        }

        void get_target_angle(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            (void)request;
            std::string message;
            // Use the same localization routine to compute angle (this may broadcast TF as well)
            bool ok = perform_localization(message);
            if (!ok) {
                response->success = false;
                response->message = message;
                return;
            }
            // Return the latest computed angle in radians as the message
            response->success = true;
            response->message = std::to_string(latest_target_angle_);
        }

        void process_data(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            (void)request;
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Processing data to localize target...");
            std::string message;
            response->success = perform_localization(message);
            response->message = message;
        }

        // void process_data(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        //                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        //     (void)request;
        //     RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Processing data to localize target...");
        //     if (right_camera_target_.header.stamp.sec == 0) {
        //         RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for camera to detect targets...");
        //         response->success = false;
        //         response->message = "No right camera target data.";
        //         return;
        //     }
        //     if (latest_lidar_scan_.header.stamp.sec == 0) {
        //         RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for LIDAR scan data...");
        //         response->success = false;
        //         response->message = "No LIDAR scan data.";
        //         return;
        //     }
        //     // Ignore old data
        //     auto now = this->now();
        //     RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Current time: %f seconds.", now.nanoseconds()/1000000000.0);
        //     RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Right camera target time: %f seconds.", rclcpp::Time(right_camera_target_.header.stamp).seconds());
        //     if ((now - rclcpp::Time(right_camera_target_.header.stamp)).seconds() > tf_timeout_) {
        //         RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Right camera target data is too old.");
        //         response->success = false;
        //         response->message = "Right camera target data is too old.";
        //         return;
        //     }
        //     if ((now - rclcpp::Time(latest_lidar_scan_.header.stamp)).seconds() > tf_timeout_) {
        //         RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] LIDAR scan data is too old.");
        //         response->success = false;
        //         response->message = "LIDAR scan data is too old.";
        //         return;
        //     }
        //     // Transform camera target to LIDAR frame
        //     geometry_msgs::msg::PoseStamped target_pose = compute_pose_from_camera(right_camera_target_, right_camera_info_);
        //     target_pose = transform_pose_to_lidar_frame(target_pose, lidar_frame_);
        //     RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Transformed target pose to LIDAR frame.");
        //     if (target_pose.header.frame_id != lidar_frame_) {
        //         RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Failed to transform target angle to LIDAR frame.");
        //         response->success = false;
        //         response->message = "Failed to transform target angle to LIDAR frame.";
        //         return;
        //     }
        //     int target_index = find_target_in_lidar_scan(target_pose, latest_lidar_scan_);
        //     if (target_index >= 0) {
        //         broadcast_target_tf(latest_lidar_scan_.angle_min + target_index * latest_lidar_scan_.angle_increment, latest_lidar_scan_.ranges[target_index], lidar_frame_);
        //         response->success = true;
        //         response->message = "Target localized and TF broadcasted.";
        //     } else {
        //         RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Target not found in LIDAR scan.");
        //         response->success = false;
        //         response->message = "Target not found in LIDAR scan.";
        //     }
        // }

        double compute_angle_from_camera(vision_msgs::msg::Detection2D & target, sensor_msgs::msg::CameraInfo & cam_info, const std::string & camera_frame){
            // Compute angle of the target relative to the camera frame
            // Use OpenCV to correct distortion
            // Construct Camera Matrix
            cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
            double fx = cam_info.k[0];
            double fy = cam_info.k[4];
            double cx = cam_info.k[2];
            double cy = cam_info.k[5];

            // Construct Distortion Coefficients
            cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);

            if (fx == 0.0) {
                 // Fallback if camera info invalid using values from camera_left_info.yaml
                 fx = 438.783367;
                 cx = 305.593336;
                 fy = 437.302876;
                 cy = 243.738352;
                 
                 dist_coeffs.at<double>(0,0) = -0.361976;
                 dist_coeffs.at<double>(0,1) = 0.110510;
                 dist_coeffs.at<double>(0,2) = 0.001014;
                 dist_coeffs.at<double>(0,3) = 0.000505;
                 dist_coeffs.at<double>(0,4) = 0.000000;

                 RCLCPP_WARN_ONCE(this->get_logger(), "Camera info invalid (fx=0). Using default fallback values.");
            } else {
                 for(size_t i=0; i<cam_info.d.size() && i<5; ++i) {
                     dist_coeffs.at<double>(0,i) = cam_info.d[i];
                 }
            }

            camera_matrix.at<double>(0,0) = fx; 
            camera_matrix.at<double>(1,1) = fy; 
            camera_matrix.at<double>(0,2) = cx; 
            camera_matrix.at<double>(1,2) = cy;
            camera_matrix.at<double>(2,2) = 1.0;

            // Define the point in image coordinates
            std::vector<cv::Point2d> points;
            points.push_back(cv::Point2d(target.bbox.center.position.x, target.bbox.center.position.y));

            std::vector<cv::Point2d> undistorted_points;
            
            // undistortPoints returns points in normalized coordinates (x', y') 
            // where x' = (u - cx)/fx, but corrected for distortion.
            // P_norm = (x', y', 1)
            cv::undistortPoints(points, undistorted_points, camera_matrix, dist_coeffs);

            if (undistorted_points.empty()) {
                RCLCPP_WARN(this->get_logger(), "UndistortPoints returned empty result");
                return 0.0;
            }

            double x_norm = undistorted_points[0].x;
            // The angle is simply atan(x_norm) because z is normalized to 1 in this projection
            double angle = atan(x_norm);
            
            //RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] ++++ %s", target.header.frame_id.c_str());
            
            double offset = 0.0;
            if (camera_frame == left_camera_frame_) {
                offset = left_angle_offset_deg_;
            } else if (camera_frame == right_camera_frame_) {
                offset = right_angle_offset_deg_;
            }
            
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] ++++ %f", offset);
            angle += offset * M_PI / 180.0;
            
            publish_camera_line(angle, target.header.frame_id);

            return angle;            
        }

        double transform_angle_to_lidar_frame(double angle_in_camera, const std::string & camera_frame, const std::string & lidar_frame, const rclcpp::Time & timestamp = rclcpp::Time()) {
            // Determine which offset to apply based on frame name
            double selected_offset_deg = 0.0;
            
            // Apply specific offset correction
            double offset_rad = selected_offset_deg * M_PI / 180.0;
            
            // Transform angle from camera frame to LIDAR frame using TF2
            geometry_msgs::msg::Vector3Stamped vec_in_camera, vec_in_lidar;
            vec_in_camera.header.frame_id = camera_frame;
            vec_in_camera.header.stamp = timestamp.nanoseconds() == 0 ? this->now() : timestamp;
            vec_in_camera.vector.x = sin(angle_in_camera);
            vec_in_camera.vector.y = 0.0;
            vec_in_camera.vector.z = cos(angle_in_camera);

            try {
                if (camera_frame == left_camera_frame_) {
                    vec_in_lidar = tf_cam_left_buffer_->transform(
                        vec_in_camera,
                        lidar_frame
                    );
                } else if (camera_frame == right_camera_frame_) {
                    vec_in_lidar = tf_cam_right_buffer_->transform(
                        vec_in_camera,
                        lidar_frame
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Unknown camera frame for angle transformation: %s", camera_frame.c_str());
                    return NAN;
                }
                //make the distance egal 0.5
                vec_in_lidar.vector.x = vec_in_lidar.vector.x;
                vec_in_lidar.vector.y = vec_in_lidar.vector.y;
                vec_in_lidar.vector.z = vec_in_lidar.vector.z;

                geometry_msgs::msg::PoseStamped target_pose;
                target_pose.header = vec_in_lidar.header;
                target_pose.pose.position.x = vec_in_lidar.vector.x;
                target_pose.pose.position.y = vec_in_lidar.vector.y;
                target_pose.pose.position.z = vec_in_lidar.vector.z;

                //publish_search_zone_marker(target_pose, 0.1);

                RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Vector in LIDAR frame: (%.2f, %.2f, %.2f)", vec_in_lidar.vector.x, vec_in_lidar.vector.y, vec_in_lidar.vector.z);
                double angle_in_lidar = atan2(vec_in_lidar.vector.y, vec_in_lidar.vector.x);
                
                // Add the tuning offset
                angle_in_lidar += offset_rad;
                
                return angle_in_lidar;          
            } catch (tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] TF2 Transform Error in angle transformation: %s", ex.what());
                return NAN;
            }
        }


        geometry_msgs::msg::PoseStamped compute_pose_from_camera(vision_msgs::msg::Detection2D & target, sensor_msgs::msg::CameraInfo & cam_info){
            geometry_msgs::msg::PoseStamped pose;
            pose.header = target.header;

            double cx = cam_info.k[2]; // Center point x
            double fx = cam_info.k[0]; // Focal length x
            // Compute direction vector from camera to target
            double angle = atan2((target.bbox.center.position.x - cx), fx);
            // Compute distance from camera
            double distance = target_width_m_ * fx / (target.bbox.size_x);

            // Set pose position
            pose.pose.position.x = distance * sin(angle);
            pose.pose.position.y = 0.0;
            pose.pose.position.z = distance * cos(angle);

            //publish search zone marker
            //publish_search_zone_marker(pose, 0.1);

            return pose;
        }

        geometry_msgs::msg::PoseStamped compute_pose_from_camera_angle(vision_msgs::msg::Detection2D & target, sensor_msgs::msg::CameraInfo & cam_info, double angle_from_cam, const std::string & frame_id){
            geometry_msgs::msg::PoseStamped pose;
            pose.header = target.header;

            double cx = cam_info.k[2]; // Center point x
            double fx = cam_info.k[0]; // Focal length x
            // Compute direction vector from camera to target
            double angle = angle_from_cam;
            // Compute distance from camera
            double distance = 0.9 * target_width_m_ * fx / (target.bbox.size_x);

            // Set pose position
            pose.pose.position.x = distance * sin(angle);
            pose.pose.position.y = 0.0;
            pose.pose.position.z = distance * cos(angle);

            //publish search zone marker
            publish_estimate_cam_pose(pose, 0.1, frame_id);

            return pose;
        }

        double compute_lidar_angle_from_camera_pose(geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id){
            geometry_msgs::msg::PoseStamped pose_in_lidar = transform_pose_to_lidar_frame(pose, lidar_frame_);

            if (pose_in_lidar.header.frame_id != lidar_frame_) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Invalid frame_id %s for pose in lidar frame", pose_in_lidar.header.frame_id.c_str());
                return NAN;
            }
            
            return atan2(pose_in_lidar.pose.position.y, pose_in_lidar.pose.position.x);
        }

        geometry_msgs::msg::PoseStamped compute_intersection(double angle_left_lidar, double angle_right_lidar, 
                                                            const geometry_msgs::msg::TransformStamped & tf_left, 
                                                            const geometry_msgs::msg::TransformStamped & tf_right) {
            geometry_msgs::msg::PoseStamped result;
            result.header.frame_id = lidar_frame_;
            result.header.stamp = this->now();
            result.pose.position.x = NAN; // Default to invalid

            // Ray 1 (Left Camera): Origin (x1, y1), Angle a1
            double x1 = tf_left.transform.translation.x;
            double y1 = tf_left.transform.translation.y;
            double a1 = angle_left_lidar;

            // Ray 2 (Right Camera): Origin (x2, y2), Angle a2
            double x2 = tf_right.transform.translation.x;
            double y2 = tf_right.transform.translation.y;
            double a2 = angle_right_lidar;

            // Standard line intersection
            // Line 1: P = P1 + t * V1, V1 = (cos(a1), sin(a1))
            // Line 2: P = P2 + u * V2, V2 = (cos(a2), sin(a2))
            // Solve for t: t * (V1 x V2) = (P2 - P1) x V2
            
            double sin_a1 = sin(a1);
            double cos_a1 = cos(a1);
            double sin_a2 = sin(a2);
            double cos_a2 = cos(a2);

            // Cross product of direction vectors
            double det = cos_a1 * sin_a2 - sin_a1 * cos_a2;

            if (std::abs(det) > 1e-6) {
                // Not parallel
                double dx = x2 - x1;
                double dy = y2 - y1;
                
                // (P2 - P1) x V2
                // (dx, dy) x (cos(a2), sin(a2))
                double t_num = dx * sin_a2 - dy * cos_a2;
                
                double t = t_num / det;

                // Intersection point
                double ix = x1 + t * cos_a1;
                double iy = y1 + t * sin_a1;
                
                // Optionally check if intersection is in front of both cameras (t > 0 and u > 0)
                // for u: (P2 - P1) x V1 / det
                // double u_num = dx * sin_a1 - dy * cos_a1;
                // double u = u_num / det;

                // Simple validation: Point must be somewhat reasonable
                result.pose.position.x = ix;
                result.pose.position.y = iy;
                result.pose.position.z = 0.0;
                
                // Orient the pose towards the intersection? Or keep 0.
                result.pose.orientation.w = 1.0;
            } else {
                 RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Rays are parallel, cannot compute intersection.");
            }

            publish_search_zone_marker(result, 0.05);

            return result;
        }

        //TODO : FIX THIS
        geometry_msgs::msg::PoseStamped transform_pose_to_lidar_frame(geometry_msgs::msg::PoseStamped & cam_pose, const std::string & lidar_frame) {
            // Transform pose from camera frame to LIDAR frame using TF2
            geometry_msgs::msg::PoseStamped pose_in_lidar;
            try {
                if (cam_pose.header.frame_id == left_camera_frame_) {
                    pose_in_lidar = tf_cam_left_buffer_->transform(
                        cam_pose,
                        lidar_frame
                    );
                } else if (cam_pose.header.frame_id == right_camera_frame_) {
                    pose_in_lidar = tf_cam_right_buffer_->transform(
                        cam_pose,
                        lidar_frame
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Unknown camera frame for pose transformation: %s", cam_pose.header.frame_id.c_str());
                    return cam_pose;
                }
                pose_in_lidar.header.stamp = this->now();
                pose_in_lidar.header.frame_id = lidar_frame_;
                //publish_search_zone_marker(pose_in_lidar, 0.05);
                return pose_in_lidar;          
            } catch (tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] TF2 Transform Error in pose transformation: %s", ex.what());
                return cam_pose;
            }
        }

        /**
         * @brief [NOT IN USE]Transforms a Detection3D target from a camera frame to the LIDAR frame. 
         * Caller must check the frame_id of the returned target to ensure successful transformation.
         * @note just in case we have 3D pose estimation from the camera, so not in use now.
         * 
         * @param cam_target The target detection in the camera frame.
         * @param lidar_frame The target frame to transform the detection into (LIDAR frame).
         * @return vision_msgs::msg::Detection3D The transformed target detection in the LIDAR frame.
         */
        vision_msgs::msg::Detection3D transform_target_to_lidar_frame(vision_msgs::msg::Detection3D & cam_target, const std::string & lidar_frame) {
            // Transform target position to LIDAR frame
            vision_msgs::msg::Detection3D lidar_target = cam_target;

            // Stamp the pose of input target
            geometry_msgs::msg::PoseStamped pose_in_camera, pose_in_lidar;
            pose_in_camera.header = cam_target.header;
            pose_in_camera.pose = cam_target.bbox.center;
            
            try {
                if (cam_target.header.frame_id == left_camera_frame_) {
                    pose_in_lidar = tf_cam_left_buffer_->transform(
                        pose_in_camera,
                        lidar_frame
                    );
                } else if (cam_target.header.frame_id == right_camera_frame_) {
                    pose_in_lidar = tf_cam_right_buffer_->transform(
                        pose_in_camera,
                        lidar_frame
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Unknown camera frame: %s", cam_target.header.frame_id.c_str());
                    // Upper-level function should check frame id of the returned target
                    return lidar_target;
                }
                // Update the target with transformed pose
                lidar_target.header.frame_id = lidar_frame;
                lidar_target.header.stamp = this->now();
                lidar_target.bbox.center = pose_in_lidar.pose;  
                return lidar_target;          
            } catch (tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] TF2 Transform Error: %s", ex.what());
                // Upper-level function should check frame id of the returned target
                return lidar_target;
            }
        }

        /**
         * @brief Find target index in LIDAR scan data based on angle. Return the index of the closest valid point.
         * 
         * @param angle The angle (in radians) to search for the target in LIDAR frame.
         * @param scan The LIDAR scan data.
         * @return int The index of the target in the LIDAR scan ranges, or -1 if not found.
         */
        int find_target_in_lidar_scan(double angle, sensor_msgs::msg::LaserScan & scan) {
            const double search_angle_tolerance_ = 10*3.141592654/180.0; // 10 degrees in radians
            const int min_valid_lidar_points_ = 3;      // Minimum pixel needed to confirm target detection
            const int max_valid_lidar_points_ = 20;   // Maximum pixel to avoid false positives
            const double max_lidar_distance_m_ = 1.5;    // Maximum distance to consider LIDAR points valid
            const double min_lidar_distance_m_ = 0.2;    // Minimum distance to consider LIDAR points valid

            publish_search_zone_marker(angle, search_angle_tolerance_, min_lidar_distance_m_, max_lidar_distance_m_);

            int index_min = std::round((angle - search_angle_tolerance_ - scan.angle_min) / scan.angle_increment);
            int index_max = std::round((angle+search_angle_tolerance_ - scan.angle_min) / scan.angle_increment);
            int index_center = std::round((angle - scan.angle_min) / scan.angle_increment);
            if (index_min < 0){
                index_min = 0;
            }
            if (index_max >= static_cast<int>(scan.ranges.size())){
                index_max = scan.ranges.size() - 1;
            }

            int pixel_detected = 0;
            
            std::array<int, max_valid_lidar_points_> index_detected = {0};
            int target_index = 0;

            // Perform sweep in LIDAR scan data within the angle range from the middle outwards
            // Perform sweep in LIDAR scan data within the angle range from the middle outwards
            for (int offset = 0; offset <= (index_max - index_min)/2; ++offset){
                // Check right side
                int i_right = index_center + offset;
                if (i_right <= index_max){
                    double distance = scan.ranges[i_right];
                    if (distance >= min_lidar_distance_m_ && distance <= max_lidar_distance_m_){
                        if (pixel_detected < max_valid_lidar_points_){
                            index_detected[pixel_detected] = i_right;
                            pixel_detected++;
                        }
                    }
                }
                // Check left side
                int i_left = index_center - offset;
                if (i_left >= index_min){
                    double distance = scan.ranges[i_left];
                    if (distance >= min_lidar_distance_m_ && distance <= max_lidar_distance_m_){
                        if (pixel_detected < max_valid_lidar_points_){
                            index_detected[pixel_detected] = i_left;
                            pixel_detected++;
                        }
                    }
                }
            }
            // for (int i = index_min; i <= index_max; ++i){
            //     double distance = scan.ranges[i];
            //     if (distance >= min_lidar_distance_m_ && distance <= max_lidar_distance_m_){
            //         if (pixel_detected < max_valid_lidar_points_){
            //             index_detected[pixel_detected] = i;
            //             pixel_detected++;
            //         }
            //     }
            // }
            RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Detected %d valid LIDAR points for target search.", pixel_detected);
            if (pixel_detected >= min_valid_lidar_points_){
                // Find minimum distance index among detected points
                double min_distance = scan.range_max + 1.0;
                for (int j = 0; j < std::min(pixel_detected, max_valid_lidar_points_); j++){
                    double distance = scan.ranges[index_detected[j]];
                    if (distance < min_distance){
                        min_distance = distance;
                        target_index = index_detected[j];
                    }
                }
                return target_index;
            } else {    
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Not enough valid LIDAR points detected for target.");
                return -1; // Indicate no valid target found
            }
        }

        /**
         * @brief Search in LIDAR scan data around the target pose to find a valid point.
         *        Point closest to the center (the target pose) will be prioritized.
         * 
         * @param target_pose Pose in LiDAR frame
         * @param scan LiDAR scan data
         * @return int Index of the target in LiDAR scan ranges, or -1 if not found.
         */
        int find_target_in_lidar_scan(geometry_msgs::msg::PoseStamped & target_pose, sensor_msgs::msg::LaserScan & scan) {
            const double tol_radius = 0.1; // Meters
            const int min_valid_lidar_points_ = 3; // Minimum pixel needed to confirm target detection
            const int max_valid_lidar_points_ = 15; // Maximum pixel to avoid false positives
            double target_angle = atan2(target_pose.pose.position.x, target_pose.pose.position.y) + M_PI/2;
            double target_distance = sqrt(
                target_pose.pose.position.x * target_pose.pose.position.x +
                target_pose.pose.position.y * target_pose.pose.position.y
            );
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Searching for target at angle: %.3f radians, distance: %.3f meters", target_angle, target_distance);

            //publish_search_zone_marker(target_pose, tol_radius);

            int index_min = std::round((target_angle - sin(tol_radius / target_distance) - scan.angle_min) / scan.angle_increment);
            int index_max = std::round((target_angle + sin(tol_radius / target_distance) - scan.angle_min) / scan.angle_increment);
            // int index_center = std::round((target_angle - scan.angle_min) / scan.angle_increment);
            if (index_min < 0){
                index_min = 0;
            }
            if (index_max >= static_cast<int>(scan.ranges.size())){
                index_max = scan.ranges.size() - 1;
            }
            int pixel_detected = 0;
            
            std::array<int, max_valid_lidar_points_> index_detected = {0};
            int target_index = 0;

            // Perform sweep in LIDAR scan data
            for (int i = index_min; i <= index_max; ++i){
                double distance = scan.ranges[i];
                double angle = scan.angle_min + i * scan.angle_increment;
                double x = distance * cos(angle);
                double y = distance * sin(angle);
                // See if point is within tolerance radius of target pose
                if ((pow(x - target_pose.pose.position.x, 2) + pow(y - target_pose.pose.position.y, 2)) <= (pow(tol_radius,2))){
                    if (pixel_detected < max_valid_lidar_points_){
                        index_detected[pixel_detected] = i;
                        pixel_detected++;
                    }
                }
            }
            RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Detected %d valid LIDAR points for target search.", pixel_detected);
            if (pixel_detected >= min_valid_lidar_points_){
                // Find point closest to target distance among detected points
                double min_distance_diff = scan.range_max + 1.0;
                for (int j = 0; j < std::min(pixel_detected, max_valid_lidar_points_); j++){
                    double distance = scan.ranges[index_detected[j]];
                    double distance_diff = fabs(distance - target_distance);
                    if (distance_diff < min_distance_diff){
                        min_distance_diff = distance_diff;
                        target_index = index_detected[j];
                    }
                }
                return target_index;
            } else {    
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Not enough valid LIDAR points detected for target.");
                return -1; // Indicate no valid target found
            }
        }

        void broadcast_target_tf(double angle, float distance, const std::string & lidar_frame) {
            geometry_msgs::msg::TransformStamped target_tf;
            target_tf.header.stamp = this->now();
            target_tf.header.frame_id = lidar_frame;
            target_tf.child_frame_id = "target_trash";
            target_tf.transform.translation.x = distance * cos(angle);
            target_tf.transform.translation.y = distance * sin(angle);
            target_tf.transform.translation.z = 0.06;
            target_tf.transform.rotation.x = 0.0;
            target_tf.transform.rotation.y = 0.0;
            target_tf.transform.rotation.z = 0.0;
            target_tf.transform.rotation.w = 1.0;

            tf_static_broadcaster_->sendTransform(target_tf);
        }

        void clear_target_tf(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            (void)request;
            geometry_msgs::msg::TransformStamped target_tf;
            target_tf.header.stamp = this->now();
            target_tf.header.frame_id = lidar_frame_;
            target_tf.child_frame_id = "target_trash";
            target_tf.transform.translation.x = 0.0;
            target_tf.transform.translation.y = 0.0;
            target_tf.transform.translation.z = 0.0;
            target_tf.transform.rotation.x = 0.0;
            target_tf.transform.rotation.y = 0.0;
            target_tf.transform.rotation.z = 0.0;
            target_tf.transform.rotation.w = 1.0;

            tf_static_broadcaster_->sendTransform(target_tf);
            response->success = true;
            response->message = "Target TF cleared.";
        }

        void publish_search_zone_marker(double angle, double tolerance, double /*min_dist*/, double max_dist) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = lidar_frame_;
            marker.header.stamp = this->now();
            marker.ns = "search_zone_angular";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            // Draw cone
            geometry_msgs::msg::Point p;
            p.x = 0; p.y = 0; p.z = 0;
            marker.points.push_back(p);
            
            p.x = max_dist * cos(angle - tolerance);
            p.y = max_dist * sin(angle - tolerance);
            marker.points.push_back(p);
            
            p.x = max_dist * cos(angle + tolerance);
            p.y = max_dist * sin(angle + tolerance);
            marker.points.push_back(p);
            
            p.x = 0; p.y = 0; p.z = 0;
            marker.points.push_back(p);

            marker_publisher_->publish(marker);
        }

        void publish_search_zone_marker(geometry_msgs::msg::PoseStamped target_pose, double radius) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = lidar_frame_;
            marker.header.stamp = this->now();
            marker.ns = "search_zone_radial";
            marker.id = 1;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = target_pose.pose;
            marker.scale.x = radius * 2;
            marker.scale.y = radius * 2;
            marker.scale.z = 0.01;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_publisher_->publish(marker);
        }

        void publish_estimate_cam_pose(geometry_msgs::msg::PoseStamped target_pose, double radius, const std::string & frame_id) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = this->now();
            marker.ns = "estimate_cam_pose";
            marker.id = 1;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = target_pose.pose;
            marker.scale.x = radius * 2;
            marker.scale.y = radius * 2;
            marker.scale.z = 0.01;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_publisher_->publish(marker);
        }

        void publish_camera_line(double angle, const std::string & frame_id) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = this->now();
            marker.ns = "camera_sight_line";
            
            // Use different IDs for left/right frames to avoid flickering if both publish
            if (frame_id == left_camera_frame_) marker.id = 2;
            else if (frame_id == right_camera_frame_) marker.id = 3;
            else marker.id = 4; // unknown

            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0; // Cyan/Light Blue

            geometry_msgs::msg::Point p_start;
            p_start.x = 0; p_start.y = 0; p_start.z = 0;
            marker.points.push_back(p_start);

            // Removed incorrect yaw subtraction logic here.
            // The marker is published IN the camera frame, so the angle (which is relative to the camera)
            // is already effectively locally correct for visualization in that frame.
            
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Camera line angle: %f", angle);

            double line_len = 2.0; // 2 meters visualization
            geometry_msgs::msg::Point p_end;
            // In standard optical frame: Z is forward, X is right, Y is down.
            // Angle is computed as atan2(x, z).
            // So x = dist * sin(angle), z = dist * cos(angle).
            p_end.x = line_len * sin(angle);
            p_end.y = 0.0;
            p_end.z = line_len * cos(angle);
            marker.points.push_back(p_end);

            marker_publisher_->publish(marker);
        }

        rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr camera_left_target_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_left_info_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr camera_right_target_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_right_info_subscriber_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_subscriber_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        vision_msgs::msg::Detection2D left_camera_target_;
        vision_msgs::msg::Detection2D right_camera_target_;
        sensor_msgs::msg::LaserScan latest_lidar_scan_;
        double latest_target_distance_ = 0.0;
        double latest_target_angle_ = 0.0;

        // Services to publish and clear target TF
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_target_tf_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_target_tf_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_target_angle_service_;

        std::string left_camera_frame_;
        std::string right_camera_frame_;
        std::string lidar_frame_;

        const double target_width_m_ = 0.06; // Approximate width of the target trash in meters

        sensor_msgs::msg::CameraInfo left_camera_info_;
        sensor_msgs::msg::CameraInfo right_camera_info_;

        std::unique_ptr<tf2_ros::Buffer> tf_cam_left_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_left_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_cam_right_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_right_listener_{nullptr};
        double tf_timeout_;
        double left_angle_offset_deg_;
        double right_angle_offset_deg_;

        bool auto_publish_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrashLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}