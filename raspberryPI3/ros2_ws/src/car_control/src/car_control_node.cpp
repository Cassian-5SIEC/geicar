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
        steeringPwmCmd = STOP;
    

        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);

        

    subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
    "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

    subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
    "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

    subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
    "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));

    subscription_us_emergency_ = this->create_subscription<interfaces::msg::Ultrasonic>(
    "us_data", 10, std::bind(&car_control::EmergencyCallback, this, _1));


        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
                            "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

        
        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

    
private:

    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
    *
    * This function is called when a message is published on the "/joystick_order" topic
    * 
    */
    void joystickOrderCallback(const interfaces::msg::JoystickOrder::SharedPtr joyOrder) {

        if (joyOrder->start != start){
            start = joyOrder->start;

            if (start)
                RCLCPP_INFO(this->get_logger(), "START");
            else 
                RCLCPP_INFO(this->get_logger(), "STOP");
        }
        

        if (joyOrder->mode != mode && joyOrder->mode != -1){ //if mode change
            mode = joyOrder->mode;

            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            }else if (mode==1){
                RCLCPP_INFO(this->get_logger(), "Switching to AUTONOMOUS Mode");
            }else if (mode==2){
                RCLCPP_INFO(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
                startSteeringCalibration();
            }
        }
        
        if (mode == 0 && start){  //if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
            requestedThrottle = joyOrder->throttle;
            requestedSteerAngle = joyOrder->steer;
            reverse = joyOrder->reverse;
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
        motorsOrder.steering_angle = steeringVal;

        if (!start){    //Car stopped or Emergency Stop

            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            //steeringPwmCmd = STOP;


        }else{ //Car started

            //Manual Mode
            if (mode==0){
                
                manualPropulsionCmd(requestedThrottle, reverse, leftRearPwmCmd,rightRearPwmCmd);

                //steeringCmd(requestedSteerAngle,currentAngle, steeringPwmCmd);


            //Autonomous Mode
            } else if (mode==1){
                //...
            }
        }

        // Forward + straight limits
        //if (!reverse && EmergencyStop[1]) stop = true;
//
        //// Reverse + straight limits
        //else if (reverse && EmergencyStop[4]) stop = true;

        if (!reverse){
            for (int i=0; i<3; i++){
                if (EmergencyStop[i]){
                    stop = true;
                    break;
                }
            }
        }

        else{
            for (int i=3; i<6; i++){
                if (EmergencyStop[i]){
                    stop = true;
                    break;
                }
            }
        }

        //// Forward + steering right
        //else if (!reverse && EmergencyStop[0] && motorsOrder.steering_angle > STOP)
        //    stop = true;
//
        //// Forward + steering left
        //else if (!reverse && EmergencyStop[2] && motorsOrder.steering_angle < STOP)
        //    stop = true;
//
        //// Reverse + steering left
        //else if (reverse && EmergencyStop[5] && motorsOrder.steering_angle < STOP)
        //    stop = true;
//
        //// Reverse + steering right (rear right)
        //else if (reverse && EmergencyStop[3] && motorsOrder.steering_angle > STOP)
        //    stop = true;

        if (stop) {
            leftRearPwmCmd  = STOP;
            rightRearPwmCmd = STOP;
        }else{

            //Send order to motors
            motorsOrder.left_rear_pwm = leftRearPwmCmd;
            motorsOrder.right_rear_pwm = rightRearPwmCmd;
            
            // steering already computed above
            currentAngle = requestedSteerAngle;
        }

        publisher_can_->publish(motorsOrder);
    }


    /* Start the steering calibration process :
    *
    * Publish a calibration request on the "/steering_calibration" topic
    */
    void startSteeringCalibration(){

        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);
    }


    /* Function called by "steering_calibration" service
    * 1. Switch to calibration mode
    * 2. Call startSteeringCalibration function
    */
    void steeringCalibration([[maybe_unused]] std_srvs::srv::Empty::Request::SharedPtr req,
                            [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res)
    {

        mode = 2;    //Switch to calibration mode
        RCLCPP_WARN(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
        startSteeringCalibration();
    }
    

    /* Manage steering calibration process [callback function]  :
    *
    * This function is called when a message is published on the "/steering_calibration" topic
    */
    void steeringCalibrationCallback (const interfaces::msg::SteeringCalibration::SharedPtr calibrationMsg){

        if (calibrationMsg->in_progress == true && calibrationMsg->user_need == false){
        RCLCPP_INFO(this->get_logger(), "Steering Calibration in progress, please wait ....");

        } else if (calibrationMsg->in_progress == true && calibrationMsg->user_need == true){
            RCLCPP_WARN(this->get_logger(), "Please use the buttons (L/R) to center the steering wheels.\nThen, press the blue button on the NucleoF103 to continue");
        
        } else if (calibrationMsg->status == 1){
            RCLCPP_INFO(this->get_logger(), "Steering calibration [SUCCESS]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        
        } else if (calibrationMsg->status == -1){
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        }
    
    }

    /*EMERGENCY STOP*/
        void EmergencyCallback(const interfaces::msg::Ultrasonic::SharedPtr USMsg) {// in centemetre
            /*I think we have to do some math to get the distance of the obstecle but i'm simplifing it now and saying it gives us directly a distance*/
            EmergencyStop[0] = (USMsg->front_left  < 50);
            EmergencyStop[1] = (USMsg->front_center < 50);
            EmergencyStop[2] = (USMsg->front_right < 50);
            EmergencyStop[3] = (USMsg->rear_right  < 50);
            EmergencyStop[4] = (USMsg->rear_center < 50);
            EmergencyStop[5] = (USMsg->rear_left   < 50);
    }


    
    // ---- Private variables ----

    //General variables
    bool start;
    bool stop = false;
    // EmergencyStop indices:
    // 0: front_left, 1: front_center, 2: front_right,
    // 3: rear_right, 4: rear_center, 5: rear_left
    std::array<bool, 6> EmergencyStop{}; // all false initially
    int mode;    //0 : Manual    1 : Auto    2 : Calibration

    
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
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_us_emergency_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Steering calibration Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_calibration_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_control>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}