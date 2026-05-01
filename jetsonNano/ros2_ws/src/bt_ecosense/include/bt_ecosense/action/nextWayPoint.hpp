#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Important for Quaternion conversion
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>

using namespace BT;

struct Waypoint {
    double x, y, theta;
};

class GetWaypointAtIndex : public StatefulActionNode
{
public:
    GetWaypointAtIndex(const std::string& name, const NodeConfig& config, const std::shared_ptr<rclcpp::Node> nh)
        : StatefulActionNode(name, config), node_(nh)
    {
        // Define your loop points here
        waypoints_ = {
            {-0.847, 0.972, 1.199},   // WP 0
            {1.348, 2.033, -0.320},  // WP 1
            {3.137, 0.109, -1.915},  // WP 2
            {1.361, -1.704, -0.390}  // WP 3
            
        };
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("index_in"),
            OutputPort<int>("index_out"),
            // CHANGE: We now output the full ROS message directly
            OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose") 
        };
    }

    // --- STATEFUL LIFECYCLE ---

    NodeStatus onStart() override {
        isRunning_ = true;
        execSuccess_ = false;
        exec_thread_ = std::make_shared<std::thread>(std::bind(&GetWaypointAtIndex::processIndex, this));
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        if (isRunning_) return NodeStatus::RUNNING;
        if (exec_thread_->joinable()) exec_thread_->join();
        return execSuccess_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

    void onHalted() override {
        isRunning_ = false;
        if (exec_thread_ && exec_thread_->joinable()) exec_thread_->join();
    }

private:

    void processIndex() {
        int index = 0;
        if (!getInput("index_in", index)) index = 0;

        // Modulo protection
        if (index < 0) index = 0;
        int safe_index = index % waypoints_.size();
        
        const auto& wp = waypoints_[safe_index];

        // --- CONVERSION TO POSE STAMPED ---
        geometry_msgs::msg::PoseStamped msg;
        
        // 1. Header
        msg.header.stamp = node_->now();
        msg.header.frame_id = "map"; // Assuming your waypoints are in map frame

        // 2. Position
        msg.pose.position.x = wp.x;
        msg.pose.position.y = wp.y;
        msg.pose.position.z = 0.0;

        // 3. Orientation (Theta -> Quaternion)
        tf2::Quaternion q;
        q.setRPY(0, 0, wp.theta); // Roll, Pitch, Yaw
        msg.pose.orientation = tf2::toMsg(q);

        // 4. Output the full message to Blackboard
        setOutput("goal_pose", msg);

        // Calculate next index
        int next_index = (safe_index + 1) % waypoints_.size();
        setOutput("index_out", next_index);

        RCLCPP_INFO(node_->get_logger(), "📍 Generated Pose for WP #%d -> X:%.2f Y:%.2f", safe_index, wp.x, wp.y);

        execSuccess_ = true;
        isRunning_ = false;
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::vector<Waypoint> waypoints_;

    std::shared_ptr<std::thread> exec_thread_;
    std::atomic<bool> isRunning_;
    std::atomic<bool> execSuccess_;
};