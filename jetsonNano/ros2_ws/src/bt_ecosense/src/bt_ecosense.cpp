#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
// Behavior Plugins
#include "ecosense_arm/bt_arm_nodes.hpp"
// #include "bt_ecosense/action/trash_localization.hpp"
#include "bt_ecosense/bt_ecosense.hpp"

class BehaviorTreeExecutor : public rclcpp::Node {
    public:
        BehaviorTreeExecutor(rclcpp::NodeOptions options) : rclcpp::Node("behavior_tree_executor", options) 
        {
            cbg_one_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Reuse the bt tick timer to get out of the Constructor for initialization
            // the this->shared_from_this() only works out of constructor and is needed
            // for Node init
            timer_bt_tick_ = create_wall_timer(std::chrono::milliseconds(bt_tick_period_ms_), std::bind(&BehaviorTreeExecutor::init, this));
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
            factory_->registerNodeType<TrashLocalizationService>("TrashLocalizationService", node);
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

            main_tree_ = std::make_shared<BT::Tree>(factory_->createTree(name_main_tree_));

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
         * @brief Timer to synchronously tick the behavior tree
         */
        rclcpp::TimerBase::SharedPtr timer_bt_tick_;
        int64_t bt_tick_period_ms_ = 50;

        // Get tree from package share directory
        std::string tree_folder_path_ = ament_index_cpp::get_package_share_directory("bt_ecosense") + "/tree/";
        const std::string name_main_tree_ = "main"; 
        /**
         * @brief Callback Group One
         */
        rclcpp::CallbackGroup::SharedPtr cbg_one_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    // Create Nodes
    std::shared_ptr<BehaviorTreeExecutor> node = std::make_shared<BehaviorTreeExecutor>(options);
    // Create Executor and Spin
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}