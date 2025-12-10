#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <cstdint>
#include <cmath>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "std_srvs/srv/empty.hpp"

#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"
#include "../include/car_control/car_control_node.h"

using namespace std;
using namespace std::chrono_literals; // for PERIOD_UPDATE_CMD (1ms)
using placeholders::_1;


class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")
    {
        start = false;
        mode = 0;
        requestedThrottle = 0.0f;
        requestedSteerAngle = 0.0f;
        reverse = false;
        stop = false;
        currentAngle = 0.0f;
        leftRearPwmCmd = STOP;
        rightRearPwmCmd = STOP;
        steeringPwmCmd = SERVO_ZERO;
    

        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
        "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

        subscription_hmi_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
        "network_joystick_order", 10, std::bind(&car_control::hmiOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        subscription_speed_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "speed_command", 10, std::bind(&car_control::SpeedCallback, this, _1));

        subscription_steering_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "steer_command", 10, std::bind(&car_control::SteerCallback, this, _1));

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

    }

    
private:

    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
    *
    * This function is called when a message is published on the "/joystick_order" topic or on the "/network_joystick_order" topic
    * 
    */
    void joystickOrderCallback(const interfaces::msg::JoystickOrder::SharedPtr joyOrder) {
        if (joyOrder->start && joyOrder->start != start){
            if (start){
                RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Start sending motor orders");
                RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Input from Joystick");
            }
            start = joyOrder->start;
            if (inputSource != SOURCE_JOYSTICK){
                inputSource = SOURCE_JOYSTICK;
            }
        } 
        if (!joyOrder->start && joyOrder->start != start){
            RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Stop sending motor orders");
            start = joyOrder->start;
        }
        
        //if mode change
        if (joyOrder->mode != mode && joyOrder->mode != -1 && inputSource == SOURCE_JOYSTICK){ 
            mode = joyOrder->mode;

            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Switching to MANUAL Mode");
            }else if (mode==1){
                RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Switching to AUTONOMOUS Mode");
            }else if (mode==2){
                RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Switching to STEERING CALIBRATION Mode");
            }
        }
        
        if (mode == MODE_MANUAL && start && inputSource == SOURCE_JOYSTICK){  //if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
            requestedThrottle = joyOrder->throttle;
            requestedSteerAngle = joyOrder->steer;
            reverse = joyOrder->reverse;
        }
    }

    void hmiOrderCallback(const interfaces::msg::JoystickOrder::SharedPtr hmiOrder) {
        if (hmiOrder->start && hmiOrder->start != start){
            RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Start sending motor orders");
            RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Input from HMI");
            start = hmiOrder->start;
            if (inputSource != SOURCE_HMI){
                inputSource = SOURCE_HMI;
            }
        } 
        if (!hmiOrder->start && hmiOrder->start != start){
            RCLCPP_INFO(this->get_logger(), "[CAR_CONTROL] Stop sending motor orders");
            start = hmiOrder->start;
        }
        
        //if mode change
        if (hmiOrder->mode != mode && hmiOrder->mode != -1 && inputSource == SOURCE_HMI){ 
            mode = hmiOrder->mode;

            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode from HMI");
            }else if (mode==1){
                RCLCPP_INFO(this->get_logger(), "Switching to AUTONOMOUS Mode from HMI");
            }else if (mode==2){
                RCLCPP_INFO(this->get_logger(), "Switching to STEERING CALIBRATION Mode from HMI");
            }
        }
        
        if (mode == MODE_MANUAL && start && inputSource == SOURCE_HMI){  //if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
            requestedThrottle = hmiOrder->throttle;
            requestedSteerAngle = hmiOrder->steer;
            reverse = hmiOrder->reverse;
        }
    }

    /* Update currentAngle from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback::SharedPtr motorsFeedback){
        (void)motorsFeedback; // not used for now
        //currentAngle = motorsFeedback->steering_angle;
    }


    /* Update PWM commands : leftRearPwmCmd, rightRearPwmCmd, steeringPwmCmd
    *
    * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "car_control_node.h"]
    * 
    * In MANUAL mode, the commands depends on :
    * - requestedThrottle, reverse, requestedSteerAngle [from joystick orders]
    * - currentAngle [from motors feedback]
    */
    void updateCmd(){

        auto motorsOrder = interfaces::msg::MotorsOrder();

        // reset transient stop flag and compute steering command early so emergency checks
        // can use a defined value
        stop = false;

        int8_t steeringVal = static_cast<int8_t>(std::round(requestedSteerAngle * 127.0f));
        

        if (!start) {
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            steeringVal = SERVO_ZERO;
        } else {
            //Manual Mode
            if (mode == MODE_MANUAL){
                manualPropulsionCmd(requestedThrottle, reverse, leftRearPwmCmd,rightRearPwmCmd);
                //steeringCmd(requestedSteerAngle,currentAngle, steeringPwmCmd);
            }
        }

        //Send order to motors
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;
        motorsOrder.steering_angle = steeringVal;
        // steering already computed above
        currentAngle = requestedSteerAngle;
        publisher_can_->publish(motorsOrder);
    }

    /* ------------ Receive orders from Autonomous stack -----------*/

    void SpeedCallback(const std_msgs::msg::Float64MultiArray Msg) {
        if (mode == MODE_AUTONOMOUS){
            if((Msg.data[0]/maxSpeed) > 1 || (Msg.data[0]/maxSpeed) < -1){
                RCLCPP_INFO(this->get_logger(), "I'm bigger then 1");
                leftRearPwmCmd = 100;
            } else {
                leftRearPwmCmd = 50 + Msg.data[0]/maxSpeed *50;
            }
            if((Msg.data[1]/maxSpeed) > 1 || (Msg.data[0]/maxSpeed) < -1){
                rightRearPwmCmd = 100;
            } else {
                rightRearPwmCmd = 50 + Msg.data[1]/maxSpeed *50;
            }
        }
    }

    void SteerCallback(const std_msgs::msg::Float64MultiArray Msg) {
        if (mode == MODE_AUTONOMOUS){
            if (Msg.data[0] < STEERING_CENTER){
                requestedSteerAngle = Msg.data[0]*SERVO_FULL_LEFT/STEERING_MAX_LEFT;
            }
            else{
                requestedSteerAngle = Msg.data[0]*SERVO_FULL_RIGHT/STEERING_MAX_RIGHT;
            }
        }
    }
    
    // ---- Private variables ----

    //General variables
    bool start = false;
    bool stop = false;
    // EmergencyStop indices:
    // 0: front_left, 1: front_center, 2: front_right,
    // 3: rear_right, 4: rear_center, 5: rear_left
    std::array<bool, 6> EmergencyStop{}; // all false initially
    int mode;    //0 : Manual    1 : Auto    2 : Calibration
    const double wheel_radius = 0.095; // in meters
    double maxSpeed = 0.53/ wheel_radius; // vitesse angulaire du robot
    int inputSource = -1; // 0: joystick, 1: HMI
    
    //Motors feedback variables
    float currentAngle;

    //Manual Mode variables (with joystick control)
    bool reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    //Control variables
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;

    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_hmi_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_us_emergency_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_speed_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_steering_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_control>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}