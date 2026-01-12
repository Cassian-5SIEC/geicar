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
#include "vision_msgs/msg/detection2_d.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class TrashLocalizationNode : public rclcpp::Node 
{   
    public:
        TrashLocalizationNode() : Node("trash_localization_node"){
            // Declare parameters with default values
            this->declare_parameter<std::double_t>("update_period_in_s", 0.2);
            this->declare_parameter<std::string>("camera_left_target_topic", "/usb_cam_left/object_target");
            this->declare_parameter<std::string>("camera_right_target_topic", "/usb_cam_right/object_target");
            this->declare_parameter<std::string>("camera_left_info_topic", "/usb_cam_left/camera_info");
            this->declare_parameter<std::string>("camera_right_info_topic", "/usb_cam_right/camera_info");
            this->declare_parameter<std::string>("lidar_scan_topic", "/ld_lidar/scan");
            this->declare_parameter<std::string>("left_camera_frame", "camera_left_link");
            this->declare_parameter<std::string>("right_camera_frame", "camera_right_link");
            this->declare_parameter<std::string>("lidar_frame", "ld_lidar_link");
            this->declare_parameter<std::double_t>("tf_timeout", 3.0);

            // Get parameters
            auto update_period_ = std::chrono::duration<double>(this->get_parameter("update_period_in_s").as_double()); 
            tf_timeout_ = this->get_parameter("tf_timeout").as_double();
            left_camera_frame_ = this->get_parameter("left_camera_frame").as_string();
            right_camera_frame_ = this->get_parameter("right_camera_frame").as_string();
            lidar_frame_ = this->get_parameter("lidar_frame").as_string();

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

            // RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Trash Localization Node has been started.");
            // timer_ = this->create_wall_timer(
            //     update_period_, 
            //     std::bind(&TrashLocalizationNode::update_target_tf, this)
            // );
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

        /**
         * @brief Perform lookup in scan data to find the target detected by the camera, and broadcast its TF.
         * @note Currently only work with left camera data, can be extended to right camera similarly.
         * @note Broadcast the closest valid point in LIDAR scan as target position.
         * 
         */
        void process_data(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            (void)request;
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Processing data to localize target...");
            if (right_camera_target_.header.stamp.sec == 0) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for camera to detect targets...");
                response->success = false;
                response->message = "No right camera target data.";
                return;
            }
            if (latest_lidar_scan_.header.stamp.sec == 0) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for LIDAR scan data...");
                response->success = false;
                response->message = "No LIDAR scan data.";
                return;
            }
            // Ignore old data
            auto now = this->now();
            // RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Current time: %f seconds.", now.nanoseconds()/1000000000.0);
            // RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Right camera target time: %f seconds.", rclcpp::Time(right_camera_target_.header.stamp).seconds());
            if ((now - rclcpp::Time(right_camera_target_.header.stamp)).seconds() > tf_timeout_) {
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Right camera target data is too old.");
                response->success = false;
                response->message = "Right camera target data is too old.";
                return;
            }
            if ((now - rclcpp::Time(latest_lidar_scan_.header.stamp)).seconds() > tf_timeout_) {
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] LIDAR scan data is too old.");
                response->success = false;
                response->message = "LIDAR scan data is too old.";
                return;
            }
            // Transform camera target to LIDAR frame
            double target_angle = compute_angle_from_camera(right_camera_target_, right_camera_info_);
            target_angle = transform_angle_to_lidar_frame(target_angle, right_camera_frame_, lidar_frame_);
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Transformed target angle to LIDAR frame: %.3f radians", target_angle);
            if (std::isnan(target_angle)) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Failed to transform target angle to LIDAR frame.");
                response->success = false;
                response->message = "Failed to transform target angle to LIDAR frame.";
                return;
            }
            int target_index = find_target_in_lidar_scan(target_angle, latest_lidar_scan_);
            if (target_index >= 0) {
                broadcast_target_tf(latest_lidar_scan_.angle_min + target_index * latest_lidar_scan_.angle_increment, latest_lidar_scan_.ranges[target_index], lidar_frame_);
                response->success = true;
                response->message = "Target localized and TF broadcasted.";
            } else {
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Target not found in LIDAR scan.");
                response->success = false;
                response->message = "Target not found in LIDAR scan.";
            }
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

        double compute_angle_from_camera(vision_msgs::msg::Detection2D & target, sensor_msgs::msg::CameraInfo & cam_info){
            // Compute angle of the target relative to the camera frame
            double cx = cam_info.k[2]; // Principal point x
            double fx = cam_info.k[0]; // Focal length x
            double pixel_x = target.bbox.center.position.x;
            double angle = atan2((pixel_x - cx), fx);
            return angle;            
        }

        double transform_angle_to_lidar_frame(double angle_in_camera, const std::string & camera_frame, const std::string & lidar_frame) {
            // Transform angle from camera frame to LIDAR frame using TF2
            geometry_msgs::msg::Vector3Stamped vec_in_camera, vec_in_lidar;
            vec_in_camera.header.frame_id = camera_frame;
            vec_in_camera.header.stamp = this->now();
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
                double angle_in_lidar = atan2(vec_in_lidar.vector.x, vec_in_lidar.vector.y) + M_PI/2;
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
            double distance = target_width_m_ * fx *2 / (target.bbox.size_x);

            // Set pose position
            pose.pose.position.x = distance * sin(angle);
            pose.pose.position.y = 0.0;
            pose.pose.position.z = distance * cos(angle);
            return pose;
        }

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
            const double search_angle_tolerance_ = 90*3.141592654/180.0; // 5 degrees in radians
            const int min_valid_lidar_points_ = 3;      // Minimum pixel needed to confirm target detection
            const int max_valid_lidar_points_ = 10;   // Maximum pixel to avoid false positives
            const double max_lidar_distance_m_ = 0.5;    // Maximum distance to consider LIDAR points valid
            const double min_lidar_distance_m_ = 0.2;    // Minimum distance to consider LIDAR points valid

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
            int index_min = std::round((target_angle - sin(tol_radius / target_distance) - scan.angle_min) / scan.angle_increment);
            int index_max = std::round((target_angle + sin(tol_radius / target_distance) - scan.angle_min) / scan.angle_increment);
            int index_center = std::round((target_angle - scan.angle_min) / scan.angle_increment);
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

        rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr camera_left_target_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_left_info_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr camera_right_target_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_right_info_subscriber_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        vision_msgs::msg::Detection2D left_camera_target_;
        vision_msgs::msg::Detection2D right_camera_target_;
        sensor_msgs::msg::LaserScan latest_lidar_scan_;
        double latest_target_distance_ = 0.0;
        double latest_target_angle_ = 0.0;

        // Services to publish and clear target TF
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_target_tf_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_target_tf_service_;

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

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrashLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}