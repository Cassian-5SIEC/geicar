#include "car_control/vehicle_controller.hpp"

VehicleController::VehicleController(const double timer_period, const double timeout_duration) :
  Node{"vehicle_controller"},
  timeout_duration_{timeout_duration},
  last_velocity_time_{get_clock()->now()},
  last_steering_time_{get_clock()->now()},
  body_width_{0.0},
  body_length_{0.0},
  wheel_radius_{0.0},
  wheel_width_{0.0},
  max_steering_angle_{0.0},
  max_velocity_{0.0},
  wheel_base_{0.0},
  track_width_{0.0},
  steering_angle_{0.0},
  velocity_{0.0},
  wheel_angular_velocity_{0.0, 0.0},
  wheel_steering_angle_{0.0, 0.0} //the raison that i kept this because i'm pretty sur that the steering isn't exactly the same so i would remenber treating it when the
{
  // Declare the used parameters
  declare_parameter<double>("body_width", 0.0);
  declare_parameter<double>("body_length", 0.0);
  declare_parameter<double>("wheel_radius", 0.0);
  declare_parameter<double>("wheel_width", 0.0);
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);

  // Get parameters on startup
  get_parameter("body_width", body_width_);
  get_parameter("body_length", body_length_);
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("wheel_width", wheel_width_);
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);

  // Set the track width and wheel base
  track_width_ = 0.447;
  wheel_base_ = 0.567;
  min_velocity_ = 0.12; 

  // Subscribers
  cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&VehicleController::cmd_vel_callback, this, std::placeholders::_1));

  // Publishers
  velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/speed_command", 10);

  position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/steer_command", 10);

  // Timer loop
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&VehicleController::timer_callback, this));
}

std::pair<double, double> VehicleController::parallel_steering_angle()
{double left_wheel_angle{0.0};
  double right_wheel_angle{0.0};

 
  if (std::abs(steering_angle_) > 1e-2) {
    left_wheel_angle = steering_angle_;
    right_wheel_angle = steering_angle_;
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> VehicleController::rear_differential_velocity()
{
  double left_wheel_velocity{velocity_};
  double right_wheel_velocity{velocity_};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-1) {
    // Small artificial correction to reduce slip
    const double k = track_width_ / wheel_base_;  // or try: 0.3
    double delta = k * velocity_ * steering_angle_;

    left_wheel_velocity  = velocity_ - delta * 0.5;
    right_wheel_velocity = velocity_ + delta * 0.5;
  }

  return std::make_pair(left_wheel_velocity, right_wheel_velocity);
}

void VehicleController::timer_callback()
{
  const auto current_time{get_clock()->now()};
  const auto velocity_elapsed_time{(current_time - last_steering_time_).nanoseconds()};
  const auto steering_elapsed_time{(current_time - last_steering_time_).nanoseconds()};

  // Reset velocity to zero if timeout
  if (velocity_elapsed_time > timeout_duration_) {
    wheel_angular_velocity_ = {0.0, 0.0};
  }

  // Reset steering angle to zero if timeout
  if (steering_elapsed_time > timeout_duration_) {
    wheel_steering_angle_ = {0.0, 0.0};
  }

  // Publish steering position
  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = wheel_steering_angle_;
  position_publisher_->publish(position_msg);

  // Publish wheels velocity
  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = wheel_angular_velocity_;
  velocity_publisher_->publish(velocity_msg);
}

void VehicleController::cmd_vel_callback(const geometry_msgs::msg::Twist ::SharedPtr msg)
{
  
  last_steering_time_ = get_clock()->now();  // Update timestamp
  //steering
  if(msg->linear.x == 0 ){
    steering_angle_=0;
  }else{
  steering_angle_ = (-1)*std::atan((wheel_base_ * msg->angular.z) / msg->linear.x);

  if (steering_angle_ > max_steering_angle_) {
    steering_angle_ = max_steering_angle_;
  } else if (steering_angle_ < -max_steering_angle_) {
    steering_angle_ = -max_steering_angle_;
  }
  }

  const auto wheel_angles{parallel_steering_angle()};

  wheel_steering_angle_ = {wheel_angles.first, wheel_angles.second};

  //velocity
  
  //changed manually
   if (msg->linear.x  > max_velocity_) {
    velocity_ = max_velocity_;
  } else if (msg->linear.x  < -max_velocity_) {
    velocity_ = -max_velocity_;
  }else if (msg->linear.x  < min_velocity_ && msg->linear.x >0 ) {
    velocity_ = min_velocity_ + msg->linear.x;
  } else if (msg->linear.x  > -min_velocity_ && msg->linear.x <0) {
    velocity_ = -min_velocity_ + msg->linear.x;
  } else {
    velocity_ = msg->linear.x ;
  }
   const auto wheel_velocity{rear_differential_velocity()};

  // Convert wheel linear velocity to wheel angular velocity
  wheel_angular_velocity_ = {(wheel_velocity.first / wheel_radius_),
                             (wheel_velocity.second / wheel_radius_)};

  RCLCPP_INFO(this->get_logger(), "Speed = %f", wheel_velocity.first);

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleController>());
  rclcpp::shutdown();
  return 0;
}