#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;
/**
 * @brief IMU Calibration Node
 * This node subscribes to raw IMU data, collects acceleration data for 5 seconds,
 * calculates the mean offset, and publishes the calibration offset.
 * While stationnary and not calibrated, the acceleration.x and acceleration.y has offsets about 0 m/s²,
 * and acceleration.z has an offset about 9.81 m/s² (gravity).
 * These offsets will be calculated here and published for can_rx_node to apply.
 */
class ImuCalibration : public rclcpp::Node
{
public:
    ImuCalibration(): Node("imu_calibration")
    {
        // Create subscription
        data_raw_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            10,
            std::bind(&ImuCalibration::imu_data_callback, this, std::placeholders::_1)
        );

        // Create publisher
        data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // Create 5-second timer for calibration
        calibrate_timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            5s,
            std::bind(&ImuCalibration::calibrate, this)
        );
        // Pause the timer until first IMU data is received
        calibrate_timer_->cancel();
        
        // Initialize offsets
        mean_offset_.linear_acceleration.x = 0.0;
        mean_offset_.linear_acceleration.y = 0.0;
        mean_offset_.linear_acceleration.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "[IMU Calibration]Start to calibrate IMU. Listening for 5 seconds...");
    }

private:
    void imu_data_callback(const sensor_msgs::msg::Imu& msg)
    {
        // Accumulate data for calibration
        if (calibrated_ == false){
            if (!first_data_received_ && calibrate_timer_->is_canceled()){
                first_data_received_ = true;
                // Start the timer on first data reception
                RCLCPP_INFO(this->get_logger(), "[IMU Calibration] First IMU data received. Starting 5 seconds calibration period...");
                calibrate_timer_->reset();
            }
            mean_offset_.linear_acceleration.x += msg.linear_acceleration.x;
            mean_offset_.linear_acceleration.y += msg.linear_acceleration.y;
            mean_offset_.linear_acceleration.z += msg.linear_acceleration.z;       
            data_count_++;
        } else {
            // Prepare IMU calibrated data message
            auto imu_data_msg = sensor_msgs::msg::Imu();
            imu_data_msg.header.stamp = msg.header.stamp;

            // Apply calibration offsets
            imu_data_msg.linear_acceleration.x = msg.linear_acceleration.x - mean_offset_.linear_acceleration.x*calibrated_;
            imu_data_msg.linear_acceleration.y = msg.linear_acceleration.y - mean_offset_.linear_acceleration.y*calibrated_;
            imu_data_msg.linear_acceleration.z = msg.linear_acceleration.z - mean_offset_.linear_acceleration.z*calibrated_;

            data_publisher_->publish(imu_data_msg);
        }
        return;
    }

    // Calculate the IMU data offsets after timer expires
    void calibrate()
    {
        if (calibrated_)
            return;
        
        RCLCPP_INFO(this->get_logger(), "[IMU Calibration] Calibration complete. Apply IMU Offsets to acceleration.");

        if (data_count_ > 0)
        {
            mean_offset_.linear_acceleration.x /= data_count_;
            mean_offset_.linear_acceleration.y /= data_count_;
            mean_offset_.linear_acceleration.z /= data_count_;

            mean_offset_.linear_acceleration.z -= GRAVITY;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[IMU Calibration] No IMU data received during calibration period.");
        }
        calibrated_ = true;

        // Stop the timer
        calibrate_timer_->cancel();
        return;
    }

    // Members
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr data_raw_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr data_publisher_;
    rclcpp::TimerBase::SharedPtr calibrate_timer_;

    sensor_msgs::msg::Imu mean_offset_ = sensor_msgs::msg::Imu();

    size_t data_count_ = 0;
    bool calibrated_ = false;
    bool first_data_received_ = false;

    const float GRAVITY = 9.80665; // m/s²
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImuCalibration>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
