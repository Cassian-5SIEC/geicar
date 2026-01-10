// Standard CPP
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"

// Behavior Tree
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

#include "ecosense_arm/bt_arm_nodes.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

class BehaviorTreeExecutor : public rclcpp::Node {
    public:
        BehaviorTreeExecutor(rclcpp::NodeOptions options) : rclcpp::Node("behavior_tree_executor", options) 
        {
            cbg_one_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Reuse the bt tick timer to get out of the Constructor for initialization
            // the this->shared_from_this() only works out of constructor and is needed
            // for Node init
            timer_bt_tick_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&BehaviorTreeExecutor::init, this));
        }
        ~BehaviorTreeExecutor(){}
    private:
        void init() {
            // Cancle the timer directly (one time use)
            timer_bt_tick_->cancel();
            // Get the Shared Pointer from the node
            auto node = this->shared_from_this();

            // Create Factory
            factory_ = std::make_shared<BT::BehaviorTreeFactory>();

            // Register Nodes
            factory_->registerNodeType<ExecutePickPlace>("ExecutePickPlace", node);
            factory_->registerNodeType<CheckTargetInRange>("CheckTargetInRange", node);
            factory_->registerNodeType<RetractArm>("RetractArm", node);
            // Register Behavior Trees
            for (auto const& entry :
                std::filesystem::directory_iterator(tree_folder_path_)) {
                std::cout << entry.path() << std::endl;
                if (entry.path().extension() == ".xml") {
                    factory_->registerBehaviorTreeFromFile(entry.path().string());
                }
            }

            main_tree_ = std::make_shared<BT::Tree>(factory_->createTree("main"));

            // Groot 2
            groot_logger_ = std::make_shared<BT::Groot2Publisher>(*main_tree_);

            // Lightweight serialization
            // file_logger_ = std::make_shared<BT::FileLogger2>(*main_tree_,
            // "test.btlog");
            cout_logger_ = std::make_shared<BT::StdCoutLogger>(*main_tree_);

            timer_bt_tick_ = create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&BehaviorTreeExecutor::behaviortreeTick, this));
        }

        void behaviortreeTick() 
        { 
            main_tree_->tickOnce(); 
        }

        std::shared_ptr<BT::BehaviorTreeFactory> factory_;
        std::shared_ptr<BT::Tree> main_tree_;
        std::shared_ptr<BT::Groot2Publisher> groot_logger_;
        std::shared_ptr<BT::FileLogger2> file_logger_;
        std::shared_ptr<BT::StdCoutLogger> cout_logger_;

        /**
         * @brief Thimer to syncronisly tick the behavior tree
         */
        rclcpp::TimerBase::SharedPtr timer_bt_tick_;

        // std::string tree_folder_path_ = "/home/tree/geicar/jetsonNano/ros2_ws/src/ecosense_arm/tree/";
        // Get tree from package share directory
        std::string tree_folder_path_ = ament_index_cpp::get_package_share_directory("ecosense_arm") + "/tree/";        
        /**
         * @brief Callback Group One
         */
        rclcpp::CallbackGroup::SharedPtr cbg_one_;
};

class MockTargetBroadcaster : public rclcpp::Node {
    public:
        MockTargetBroadcaster(rclcpp::NodeOptions options) : rclcpp::Node("mock_target_broadcaster", options) 
        {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            timer_broadcast_ = create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(&MockTargetBroadcaster::broadcastTarget, this));
        }
    private:
        void broadcastTarget() {
            geometry_msgs::msg::TransformStamped transformStamped;
            geometry_msgs::msg::Point target_point;
            double x = 0.25;
            double y = 0.1;
            double z = -0.2;
            target_point.x = x;
            target_point.y = y;
            target_point.z = z;
            transformStamped.header.stamp = this->get_clock()->now();
            transformStamped.header.frame_id = "Arm_Base";
            transformStamped.child_frame_id = "target_trash";
            transformStamped.transform.translation.x = x;
            transformStamped.transform.translation.y = y;
            transformStamped.transform.translation.z = z;

            tf_broadcaster_->sendTransform(transformStamped);
        }

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_broadcast_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    // Create Nodes
    std::shared_ptr<BehaviorTreeExecutor> node = std::make_shared<BehaviorTreeExecutor>(options);
    std::shared_ptr<MockTargetBroadcaster> mock_tf_node = std::make_shared<MockTargetBroadcaster>(options);
    // Create Executor and Spin
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.add_node(mock_tf_node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
    executor.remove_node(mock_tf_node->get_node_base_interface());  
    rclcpp::shutdown();
    return 0;
}