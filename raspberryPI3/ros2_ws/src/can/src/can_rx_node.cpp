#include <stdint.h>
#include <math.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/general_data.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/gnss.hpp"
#include "interfaces/msg/system_check.hpp"

#include "../include/can/can.h"



class can_rx : public rclcpp::Node {

public:
    can_rx()
    : Node("can_rx_node")
    {
        publisher_us_ = this->create_publisher<interfaces::msg::Ultrasonic>("us_data", 10);
        publisher_imu_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        publisher_imu_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
        publisher_gnss_ = this->create_publisher<interfaces::msg::Gnss>("gnss_data", 10);
        publisher_motorsFeedback_ = this->create_publisher<interfaces::msg::MotorsFeedback>("motors_feedback", 10);
        publisher_generalData_ = this->create_publisher<interfaces::msg::GeneralData>("general_data", 10);
        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);
        publisher_systemCheck_ = this->create_publisher<interfaces::msg::SystemCheck>("system_check", 10);

        if (initCommunication()==0){
            readData(); 
            closeCommunication();
        }
    }


private:

    /* Initialize CAN communicaction :
    * 1. create socket
    * 2. bind
    */
    int initCommunication(){
        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "CAN initialization failed - Socket creation error");
            return 1;
        }

        strcpy(ifr.ifr_name, "can0" );
        ioctl(s, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "CAN initialization failed - Bind error");
            return 1;
        }

        return 0;
    }


    /* Reads data from CAN bus :
    * 1. checks frames ID
    * 2. extracts data
    * 3. publishes data on the relevant topic
    */
    int readData(){

        RCLCPP_INFO(this->get_logger(), "Reading CAN bus ...");

        while (rclcpp::ok()){
            
            nbytes = read(s, &frame, sizeof(struct can_frame));

            if (nbytes < 0 && rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Read error");
                return 1;
            }

            RCLCPP_DEBUG(this->get_logger(), "Read value (ID) : 0x%03X", frame.can_id);

            for (i = 0; i < frame.can_dlc; i++)
                RCLCPP_DEBUG(this->get_logger(), "Read value (Data) : %02X", frame.data[i]);

            
            if (frame.can_id==ID_MOTORS_DATAS){
                auto motorsFeedbackMsg = interfaces::msg::MotorsFeedback();

                //Update odometry
                motorsFeedbackMsg.left_rear_odometry = frame.data[0];
                motorsFeedbackMsg.right_rear_odometry = frame.data[1];

                //Update Motors speed
                float leftSpeedMes = (frame.data[2] << 8) + frame.data[3];
                float rightSpeedMes = (frame.data[4] << 8) + frame.data[5];

                motorsFeedbackMsg.left_rear_speed = 0.01 * leftSpeedMes ; 
                motorsFeedbackMsg.right_rear_speed = 0.01 * rightSpeedMes ; 

                //Update Steering Angle (not used now)
                //int steer = frame.data[6];  //Receive steer in [0;200]
                //motorsFeedbackMsg.steering_angle = (steer - 100.0)/100.0; //Convert steer in [-1;1]

                //Publication on topics
                RCLCPP_DEBUG(this->get_logger(), "Publishing to motors_feedback and general_data topics");
                publisher_motorsFeedback_->publish(motorsFeedbackMsg);


            } else if (frame.can_id == ID_CALIBRATION_MODE){
                auto calibrationMsg = interfaces::msg::SteeringCalibration();
                
                if (frame.data[0]==CALIBRATION_IN_PROGRESS){
                    calibrationMsg.in_progress = true;

                }else if (frame.data[0]==CALIBRATION_SUCCESS){
                    calibrationMsg.status = 1;
                }else if (frame.data[0]==CALIBRATION_FAIL){
                    calibrationMsg.status = -1;
                }

                if (frame.data[1]==CALIBRATION_USER_NEED){
                    calibrationMsg.user_need = true;
                }else{
                    calibrationMsg.user_need = false;
                }

                publisher_steeringCalibration_->publish(calibrationMsg);

            }else if (frame.can_id==ID_COMM_CHECKING){

                if (frame.data[1]==COMM_F103_ACK){
                    auto systemCheckMsg = interfaces::msg::SystemCheck();
                    systemCheckMsg.response = true;
                    systemCheckMsg.f103 = true;
                    publisher_systemCheck_->publish(systemCheckMsg);

                }else if (frame.data[1]==COMM_L476_ACK){
                    auto systemCheckMsg = interfaces::msg::SystemCheck();
                    systemCheckMsg.response = true;
                    systemCheckMsg.l476 = true;
                    publisher_systemCheck_->publish(systemCheckMsg);
                }
                
            }else if (frame.can_id==ID_US1){

                usFrontLeft = (frame.data[0]<<8) + frame.data[1];
                usFrontCenter = (frame.data[2]<<8) + frame.data[3];
                usFrontRight = (frame.data[4]<<8) + frame.data[5];

            }else if (frame.can_id==ID_US2){
                RCLCPP_DEBUG(this->get_logger(), "Publishing to us_data Topic");
                auto us_msg = interfaces::msg::Ultrasonic();

                usRearLeft = (frame.data[0]<<8) + frame.data[1];
                usRearCenter = (frame.data[2]<<8) + frame.data[3];
                usRearRight = (frame.data[4]<<8) + frame.data[5];

                us_msg.front_center = usFrontCenter;
                us_msg.front_left = usFrontLeft;
                us_msg.front_right = usFrontRight;

                us_msg.rear_center = usRearCenter;
                us_msg.rear_left = usRearLeft;
                us_msg.rear_right = usRearRight;

                publisher_us_->publish(us_msg);

            }else if (frame.can_id==ID_GPS1){
                latitude = frame.data[0] + frame.data[1]/pow(10,2) + frame.data[2]/pow(10,4) + frame.data[3]/pow(10,6) + frame.data[4]/pow(10,8);
                latitude += frame.data[5]/pow(10,10) + frame.data[6]/pow(10,12) + frame.data[7]/pow(10,14);

            }else if (frame.can_id==ID_GPS2){
                longitude = frame.data[0] + frame.data[1]/pow(10,2) + frame.data[2]/pow(10,4) + frame.data[3]/pow(10,6) + frame.data[4]/pow(10,8);
                longitude += frame.data[5]/pow(10,10) + frame.data[6]/pow(10,12) + frame.data[7]/pow(10,14);

            }else if (frame.can_id==ID_GPS3){

                altitude = frame.data[0] + frame.data[1]/pow(10,2) + frame.data[2]/pow(10,4) + frame.data[3]/pow(10,6) + frame.data[4]/pow(10,8);
                altitude += frame.data[5]/pow(10,10) + frame.data[6]/pow(10,12) + frame.data[7]/pow(10,14);
            
            }else if (frame.can_id==ID_GPS4){
                RCLCPP_DEBUG(this->get_logger(), "Publishing to gnss_data Topic");
                auto gnss_msg = interfaces::msg::Gnss();

                quality = frame.data[0];
                hAcc = (frame.data[1] << 16) + (frame.data[2] << 8) + frame.data[3];
                vAcc = (frame.data[4] << 16) + (frame.data[5] << 8) + frame.data[6];

                gnss_msg.latitude = latitude;
                gnss_msg.longitude = longitude;
                gnss_msg.altitude = altitude;
                gnss_msg.quality = quality;
                gnss_msg.hacc = hAcc;
                gnss_msg.vacc = vAcc;

                publisher_gnss_->publish(gnss_msg);

            /* Update and publish Magnetic field
            * Read values in milligauss [mG]
            * Published values in Tesla [T]
            */
            }else if (frame.can_id==ID_IMU1){    
                RCLCPP_DEBUG(this->get_logger(), "Publishing to imu/mag Topic");
                auto imu_mag_msg = sensor_msgs::msg::MagneticField();
                imu_mag_msg.header.stamp = rclcpp::Clock().now();
                
                int mag_x = (frame.data[0]<<8) + frame.data[1];
                int mag_y = (frame.data[2]<<8) + frame.data[3];
                int mag_z = (frame.data[4]<<8) + frame.data[5];

                if ((frame.data[6] & 0x4) == 1){
                    mag_x = -mag_x;
                }

                if ((frame.data[6] & 0x2) == 1){
                    mag_y = -mag_y;
                }

                if ((frame.data[6] & 0x1) == 1){
                    mag_z = -mag_z;
                }
                imu_mag_msg.magnetic_field.x = mag_x * pow(10,-7);
                imu_mag_msg.magnetic_field.y = mag_y * pow(10,-7);
                imu_mag_msg.magnetic_field.z = mag_z * pow(10,-7);

                publisher_imu_mag_->publish(imu_mag_msg); 

            /* Update Angular Velocity
            * Read values in milli degree per second [mdps]
            */
            }else if (frame.can_id==ID_IMU2){  

                ang_vel_x = (frame.data[0]<<8) + frame.data[1];
                ang_vel_y = (frame.data[2]<<8) + frame.data[3];
                ang_vel_z = (frame.data[4]<<8) + frame.data[5];

                if ((frame.data[6] & 0x4) == 1){
                    ang_vel_x = -ang_vel_x;
                }

                if ((frame.data[6] & 0x2) == 1){
                    ang_vel_y = -ang_vel_y;
                }

                if ((frame.data[6] & 0x1) == 1){
                    ang_vel_z = -ang_vel_z;
                }
            

            /* Update acceleration, and publish IMU message (acceleration and angular velocity)
            * Read values in milli g [mg] and milli degree per second [mdps]
            * Published values in [m/s²] and [rad/s]
            */
            }else if (frame.can_id==ID_IMU3){ //Update acceleration + publish acceleration and angular velocity
                // Stamp current time
                auto imu_raw_msg = sensor_msgs::msg::Imu();
                imu_raw_msg.header.stamp = rclcpp::Clock().now();

                RCLCPP_DEBUG(this->get_logger(), "Publishing to imu/raw Topic");

                int acc_x = (frame.data[0]<<8) + frame.data[1];
                int acc_y = (frame.data[2]<<8) + frame.data[3];
                int acc_z = (frame.data[4]<<8) + frame.data[5];

                if ((frame.data[6] & 0x4) == 1){
                    acc_x = -acc_x;
                }

                if ((frame.data[6] & 0x2) == 1){
                    acc_y = -acc_y;
                }

                if ((frame.data[6] & 0x1) == 1){
                    acc_z = -acc_z;
                }

                imu_raw_msg.linear_acceleration.x = acc_x * pow(9.80665,-2);    //Conversion to [m/s²]
                imu_raw_msg.linear_acceleration.y = acc_y * pow(9.80665,-2);    //Conversion to [m/s²]
                imu_raw_msg.linear_acceleration.z = acc_z * pow(9.80665,-2);    //Conversion to [m/s²]

                imu_raw_msg.angular_velocity.x = ang_vel_x * pow(1.7453,-5);    //Conversion to [rad/s]
                imu_raw_msg.angular_velocity.y = ang_vel_y * pow(1.7453,-5);    //Conversion to [rad/s]
                imu_raw_msg.angular_velocity.z = ang_vel_z * pow(1.7453,-5);    //Conversion to [rad/s]

                publisher_imu_raw_->publish(imu_raw_msg);


            //Update temperature, pressure, humidity and publish general data (battery, temperature, humidity, pressure)
            }else if (frame.can_id==ID_IMU4){ 
                RCLCPP_DEBUG(this->get_logger(), "Publishing to /general_data Topic");

                temperature = ((frame.data[0]<<8)+frame.data[1]) / 10.0;
                pressure = (frame.data[2]<<8)+frame.data[3];
                humidity = frame.data[4];
                
                auto generalDataMsg = interfaces::msg::GeneralData();

                generalDataMsg.temperature = temperature;
                generalDataMsg.pressure = pressure;
                generalDataMsg.humidity= humidity;
                generalDataMsg.battery_level = batteryLevel;

                publisher_generalData_->publish(generalDataMsg);


            //Update battery_level and publish general data (battery, temperature, humidity, pressure)
            } else if (frame.can_id == ID_BATT_LEVEL){  
                RCLCPP_DEBUG(this->get_logger(), "Publishing to /general_data Topic");

                auto generalDataMsg = interfaces::msg::GeneralData();

                //Update Battery level
                float batMes = (frame.data[0] << 8) + frame.data[1];
                batMes = batMes * (11.65 / 2794.0); //Convert to Volts

                batteryLevel = batMes;

                generalDataMsg.battery_level = batteryLevel;
                generalDataMsg.temperature = temperature;
                generalDataMsg.pressure = pressure;
                generalDataMsg.humidity= humidity;

                publisher_generalData_->publish(generalDataMsg);
            }

        }

        return 0;
    }

    int closeCommunication(){

        if (close(s) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Close communication failed - Socket close error");
            return 1;
        }
        RCLCPP_INFO(this->get_logger(), "Socket closed");
        return 0;
    }

    //CAN variables
    int s, i; 
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    //US variables
    int usFrontLeft, usFrontRight, usFrontCenter, usRearLeft, usRearRight, usRearCenter;

    //GNSS variables
    double latitude;    //[DD]
    double longitude;   //[DD]
    double altitude;    //[m]
    int quality;        
    int hAcc;           //[mm]
    int vAcc;           //[mm]

    //IMU variables : angular velocity [mdps]
    int ang_vel_x;  
    int ang_vel_y;
    int ang_vel_z;

    //General data
    float batteryLevel; //[V]
    float temperature; //[°C]
    int pressure; //[hPa]
    int humidity; //%


    //Publishers
    rclcpp::Publisher<interfaces::msg::Ultrasonic>::SharedPtr publisher_us_;
    rclcpp::Publisher<interfaces::msg::Gnss>::SharedPtr publisher_gnss_;

    rclcpp::Publisher<interfaces::msg::MotorsFeedback>::SharedPtr publisher_motorsFeedback_;
    rclcpp::Publisher<interfaces::msg::GeneralData>::SharedPtr publisher_generalData_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_raw_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_imu_mag_;

    rclcpp::Publisher<interfaces::msg::SystemCheck>::SharedPtr publisher_systemCheck_;
    
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<can_rx>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}