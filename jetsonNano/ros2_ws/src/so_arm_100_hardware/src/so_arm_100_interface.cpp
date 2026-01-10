#include "so_arm_100_hardware/so_arm_100_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace so_arm_100_controller
{
SOARM100Interface::SOARM100Interface() 
{
}

SOARM100Interface::~SOARM100Interface()
{
    if (use_serial_) {
        st3215_.end();
    }
}

CallbackReturn SOARM100Interface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    use_serial_ = hardware_info.hardware_parameters.count("use_serial") ?
        (hardware_info.hardware_parameters.at("use_serial") == "true") : false;
    
    serial_port_ = hardware_info.hardware_parameters.count("serial_port") ?
        hardware_info.hardware_parameters.at("serial_port") : "/dev/ttyACM0";
    
    serial_baudrate_ = hardware_info.hardware_parameters.count("serial_baudrate") ?
        std::stoi(hardware_info.hardware_parameters.at("serial_baudrate")) : 1000000;

    size_t num_joints = info_.joints.size();
    position_commands_.resize(num_joints, 0.0);
    position_states_.resize(num_joints, 0.0);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SOARM100Interface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SOARM100Interface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }
    return command_interfaces;
}

CallbackReturn SOARM100Interface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Activating so_arm_100 hardware interface...");

    // Load calibration
    std::string calib_file = info_.hardware_parameters.count("calibration_file") ?
        info_.hardware_parameters.at("calibration_file") : "";
        
    if (!calib_file.empty()) {
        if (!load_calibration(calib_file)) {
            RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                       "Failed to load calibration file: %s", calib_file.c_str());
        }
    } else {
        // Load default calibration file from package
        std::string default_calib_file = ament_index_cpp::get_package_share_directory("so_arm_100_hardware") + "/config/calibration.yaml";
        if (!load_calibration(default_calib_file)) {
            RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                       "Failed to load default calibration file: %s", default_calib_file.c_str());
        }
    }

    // Load initial positions
    std::string initial_position_file = ament_index_cpp::get_package_share_directory("so_arm_100_hardware") + "/config/initial_positions.yaml";

    load_initial_positions(initial_position_file);

    if (use_serial_) {
        if(!st3215_.begin(serial_baudrate_, serial_port_.c_str())) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                        "Failed to open serial port %s at baudrate %d", 
                        serial_port_.c_str(), serial_baudrate_);
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), "Failed to initialize motors");
            return CallbackReturn::ERROR;
        }

        // Initialize each servo
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            // RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
            //             "Initializing servo %d...", servo_id);
            // First ping the servo
            if (st3215_.Ping(servo_id) == -1) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "No response from servo %d during initialization", servo_id);
                return CallbackReturn::ERROR;
            }
            
            // Set to position control mode
            if (!st3215_.Mode(servo_id, 0)) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "Failed to set mode for servo %d", servo_id);
                return CallbackReturn::ERROR;
            }

            // Set the initial pose
            if (i < initial_positions_.size()) {
                int init_ticks = radians_to_ticks(initial_positions_[i], i);
                position_commands_[i] = initial_positions_[i];
            }
            RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                           "Servo %d initialized at position %f", servo_id, position_commands_[i]);
            // Read initial position and set command to match
            if (st3215_.FeedBack(servo_id) != -1) {
                int pos = st3215_.ReadPos(servo_id);
                // calibrate_servo(servo_id, pos);
                position_states_[i] = ticks_to_radians(pos, i);
                // position_commands_[i] = position_states_[i];
                // RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                //            "Servo %d initialized at position %d with radians %f", servo_id, pos, position_states_[i]);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Serial communication initialized on %s", serial_port_.c_str());
    }

    node_ = rclcpp::Node::make_shared("so_arm_100_driver");
    feedback_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "feedback", 10, std::bind(&SOARM100Interface::feedback_callback, this, std::placeholders::_1));
    command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("command", 10);

    // Add services
    calib_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "record_position",
        std::bind(&SOARM100Interface::calibration_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));
                  
    torque_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "toggle_torque",
        std::bind(&SOARM100Interface::torque_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });


    

    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SOARM100Interface::on_deactivate(const rclcpp_lifecycle::State &)
{
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            st3215_.EnableTorque(servo_id, 0);
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

void SOARM100Interface::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_feedback_msg_ = msg;
}

hardware_interface::return_type SOARM100Interface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (use_serial_ && torque_enabled_) {  // Only write if torque is enabled
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            // Convert from radians (-π to π) to servo ticks (0-4095)
            int joint_pos_cmd = radians_to_ticks(position_commands_[i], i);
            // if (servo_id == 5){
            //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
            //                 "Servo %d: Command position %.2f rad -> %d ticks", 
            //                 servo_id, position_commands_[i], joint_pos_cmd);     
            // }
            // RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
            //         "Servo %d command: %.2f rad -> %d ticks", 
            //         servo_id, position_commands_[i], joint_pos_cmd);
            
            
            if (!st3215_.RegWritePosEx(servo_id, joint_pos_cmd, 4500, 255)) {
                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                           "Failed to write position to servo %d", servo_id);
            }
        }
        st3215_.RegWriteAction();
    }

    if (command_publisher_) {
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = node_->now();
        
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            cmd_msg.name.push_back(info_.joints[i].name);
            cmd_msg.position.push_back(position_commands_[i]);
        }
        
        command_publisher_->publish(cmd_msg);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SOARM100Interface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            // Add small delay between reads
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Increased delay

            if (!torque_enabled_) {
                // When torque is disabled, only try to read position
                int raw_pos = st3215_.ReadPos(servo_id);
                if (raw_pos != -1) {
                    position_states_[i] = ticks_to_radians(raw_pos, i);
                }
                continue;  // Skip other reads
            }

            // Full feedback read when torque is enabled
            if (st3215_.FeedBack(servo_id) != -1) {
                int raw_pos = st3215_.ReadPos(servo_id);
                position_states_[i] = ticks_to_radians(raw_pos, i);
                
                double speed = -1 * st3215_.ReadSpeed(servo_id) * 2 * M_PI / 4096.0;
                double pwm = -1 * st3215_.ReadLoad(servo_id) / 10.0;
                int move = st3215_.ReadMove(servo_id);
                double temperature = st3215_.ReadTemper(servo_id);
                double voltage = st3215_.ReadVoltage(servo_id) / 10;
                double current = st3215_.ReadCurrent(servo_id) * 6.5 / 1000;
                // if (servo_id == 4){
                //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                //                 "Servo %d: raw_pos=%d (%.2f rad) speed=%.2f pwm=%.2f temp=%.1f V=%.1f I=%.3f", 
                //                 servo_id, raw_pos, position_states_[i], speed, pwm, temperature, voltage, current);
                //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Servo %d position %.2f", servo_id, position_states_[i]);     
                // }
            } else {
                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                           "Failed to read feedback from servo %d", servo_id);
            }
        }
    }
    else {
        sensor_msgs::msg::JointState::SharedPtr feedback_copy;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            feedback_copy = last_feedback_msg_;
        }

        if (feedback_copy) {
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                auto it = std::find(feedback_copy->name.begin(), feedback_copy->name.end(), info_.joints[i].name);
                if (it != feedback_copy->name.end()) {
                    size_t idx = std::distance(feedback_copy->name.begin(), it);
                    if (idx < feedback_copy->position.size()) {
                        position_states_[i] = ticks_to_radians(feedback_copy->position[idx], i);
                    }
                }
            }
        }
    }

    return hardware_interface::return_type::OK;
}

void SOARM100Interface::calibrate_servo(uint8_t servo_id, int current_pos) 
{
    size_t idx = servo_id - 1;
    // Calculate offset from current position to expected zero
    int offset = current_pos - zero_positions_[idx];
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
               "Servo %d: current=%d, zero=%d, offset=%d", 
               servo_id, current_pos, zero_positions_[idx], offset);
}

double SOARM100Interface::ticks_to_radians(int ticks, size_t servo_idx) 
{
    const std::string& joint_name = info_.joints[servo_idx].name;
    // if (servo_idx == 3){
    //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
    //            "[Ticks -> Radians] for joint %s with ID %ld: %d ticks", 
    //            joint_name.c_str(), servo_idx+1, ticks);
    // }
    if (joint_calibration_.count(joint_name) > 0) {
        const auto& calib = joint_calibration_[joint_name];

        // Convert to normalized position first (0 to 1)
        double normalized = (double)(ticks - calib.min_ticks) / calib.range_ticks;
        // Then convert to (min_pos to max_pos) in radians
        if (calib.min_pos > calib.max_pos){
            return (calib.min_pos - normalized * calib.range_pos);
        }
        // if (servo_idx == 3){
        //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
        //             "Normalized value for joint %s with ID %ld: %.4f", joint_name.c_str(), servo_idx+1, normalized);
        //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
        //         "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        // }
        return (normalized * calib.range_pos + calib.min_pos);
    }
    
    // Fallback to default calibration
    // RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
    //         "Fallback to default for joint %s with ID %ld", joint_name.c_str(), servo_idx+1);
    // RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
    //         "------------------------------------------------------------------");
    return servo_directions_[servo_idx] * 
           (ticks - zero_positions_[servo_idx]) * 2 * M_PI / 4096.0;
}

int SOARM100Interface::radians_to_ticks(double radians, size_t servo_idx) 
{
    const std::string& joint_name = info_.joints[servo_idx].name;
    double normalized = 0.0;
    // if (servo_idx == 3){
    //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
    //             "[Radians -> Ticks] for joint %s with ID %ld: %.4f radians", joint_name.c_str(), servo_idx+1, radians);
    // }
    if (joint_calibration_.count(joint_name) > 0) {
        const auto& calib = joint_calibration_[joint_name];
        // Convert from (min_pos to max_pos) in rad to normalized position (0 to 1)
        if (calib.min_pos > calib.max_pos){
            normalized = (calib.min_pos - radians) / calib.range_pos;
        } else {
            normalized = (radians - calib.min_pos) / calib.range_pos;
        }
        // if (servo_idx == 4){
        //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
        //                 "Normalized value for joint %s with ID %ld: %.4f", joint_name.c_str(), servo_idx+1, calib.min_ticks + (int)(normalized * calib.range_ticks));
        //     RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
        //         "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        // }
        // Then convert to ticks
        return calib.min_ticks + (int)(normalized * calib.range_ticks);
    }
    
    // Fallback to default calibration
    // RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
    //         "Fallback to default for joint %s with ID %ld", joint_name.c_str(), servo_idx+1);

    // RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
    //         "------------------------------------------------------------------");
    return zero_positions_[servo_idx] + 
           servo_directions_[servo_idx] * (int)(radians * 4096.0 / (2 * M_PI));
}

void SOARM100Interface::record_current_position() 
{
    std::stringstream ss;
    ss << "{";  // Start with just a curly brace
    
    bool first = true;  // To handle commas between entries
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        uint8_t servo_id = static_cast<uint8_t>(i + 1);
        
        // Add delay between reads
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Try multiple times to read the servo
        int pos = -1;
        for (int retry = 0; retry < 3 && pos == -1; retry++) {
            st3215_.FeedBack(servo_id);
            pos = st3215_.ReadPos(servo_id);
            if (pos == -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        if (!first) {
            ss << ",";
        }
        first = false;
        
        ss << "\"" << info_.joints[i].name << "\": {"
           << "\"ticks\": " << (pos != -1 ? pos : 0) << ","
           << "\"speed\": " << st3215_.ReadSpeed(servo_id) << ","
           << "\"load\": " << st3215_.ReadLoad(servo_id)
           << "}";
    }
    ss << "}";  // Close the JSON object
    
    last_calibration_data_ = ss.str();
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                "Recorded positions: %s", last_calibration_data_.c_str());
}

void SOARM100Interface::calibration_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    record_current_position();
    response->success = true;
    response->message = last_calibration_data_;
}

void SOARM100Interface::set_torque_enable(bool enable) 
{
    if (use_serial_) {
        // First set all servos
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            if (!enable) {
                // When disabling:
                // 1. Set to idle mode first
                st3215_.Mode(servo_id, 2);  // Mode 2 = idle
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 2. Disable torque
                st3215_.EnableTorque(servo_id, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 3. Double check it's disabled
                st3215_.EnableTorque(servo_id, 0);
            } else {
                // When enabling:
                // 1. Set position mode
                st3215_.Mode(servo_id, 0);  // Mode 0 = position
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 2. Enable torque
                st3215_.EnableTorque(servo_id, 1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Wait a bit to ensure commands are processed
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Update state after all servos are set
        torque_enabled_ = enable;
        
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Torque %s for all servos", enable ? "enabled" : "disabled");
    }
}

void SOARM100Interface::torque_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    bool new_state = !torque_enabled_;
    
    // Set response before changing state
    response->success = true;
    response->message = std::string("Torque ") + (new_state ? "enabled" : "disabled");
    
    // Change state after setting response
    set_torque_enable(new_state);
    
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                "Torque service called, response: %s", response->message.c_str());
}

bool SOARM100Interface::load_calibration(const std::string& filepath) 
{
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        auto joints = config["joints"];
        if (!joints) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                        "No joints section in calibration file");
            return false;
        }

        for (const auto& joint : joints) {
            std::string name = joint.first.as<std::string>();
            const auto& data = joint.second;
            
            if (!data["min"] || !data["center"] || !data["max"]) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "Missing calibration data for joint %s", name.c_str());
                continue;
            }

            JointCalibration calib;
            calib.min_ticks = data["min"]["ticks"].as<int>();
            calib.center_ticks = data["center"]["ticks"].as<int>();
            calib.max_ticks = data["max"]["ticks"].as<int>();
            calib.range_ticks = calib.max_ticks - calib.min_ticks;

            calib.min_pos = data["min"]["pos"].as<double>();
            calib.center_pos = data["center"]["pos"].as<double>();
            calib.max_pos = data["max"]["pos"].as<double>();
            calib.range_pos = abs(calib.max_pos - calib.min_pos);
            
            joint_calibration_[name] = calib;
            
        }
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Loaded calibration from %s", filepath.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                    "Failed to load calibration: %s", e.what());
        return false;
    }
}

bool SOARM100Interface::load_initial_positions(const std::string& filepath) 
{
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        auto joints = config["joints"];
        if (!joints) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                        "No joints section in initial positions file");
            return false;
        }
        int id = 0;
        for (const auto& joint : joints) {
            std::string name = joint.first.as<std::string>();
            double position = joint.second["pos"].as<double>();
            
            position_commands_[id] = position;
            id++;
            // Find joint index
            // auto it = std::find_if(info_.joints.begin(), info_.joints.end(),
            //                        [&name](const hardware_interface::ComponentInfo& joint_info) {
            //                            return joint_info.name == name;
            //                        });
            // if (it != info_.joints.end()) {
            //     size_t idx = std::distance(info_.joints.begin(), it);
            //     initial_positions_[idx] = position;
            // }
        }
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Loaded initial positions from %s", filepath.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                    "Failed to load initial positions: %s", e.what());
        return false;
    }
}

double SOARM100Interface::normalize_position(const std::string& joint_name, int ticks) 
{
    if (joint_calibration_.count(joint_name) == 0) {
        return 0.0;
    }
    
    const auto& calib = joint_calibration_[joint_name];
    double normalized = (ticks - calib.min_ticks) / calib.range_ticks;
    return std::clamp(normalized, 0.0, 1.0);
}

}  // namespace so_arm_100_controller

PLUGINLIB_EXPORT_CLASS(so_arm_100_controller::SOARM100Interface, hardware_interface::SystemInterface)

