#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class DemoPickPlaceNode : public rclcpp::Node
{
public:
    DemoPickPlaceNode() : Node("demo_pick_place_node")
    {
        RCLCPP_INFO(this->get_logger(), "Demo Pick and Place Node Initialized");
        
        this->declare_parameter("step_by_step", false);
        step_by_step = this->get_parameter("step_by_step").as_bool();
    }
    void run_demo()
    {
        // Create the MoveGroupInterface
        auto arm_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        auto gripper_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

        // Set move sequence    
        // Create an iterable container with group and target pairs
        std::vector<std::pair<moveit::planning_interface::MoveGroupInterface*, std::string>> sequence = {
            {&arm_group, "home"},
            {&gripper_group, "close_gripper"},
            {&arm_group, "default_pickup_ready"},
            {&gripper_group, "open_gripper"},
            {&arm_group, "default_pickup"},
            {&gripper_group, "close_gripper"},
            {&arm_group, "dropping"},
            {&gripper_group, "open_gripper"},
            {&gripper_group, "close_gripper"},
            {&arm_group, "home"}
        };
        while (rclcpp::ok()){
            for (auto& [group, target] : sequence)
            {
                bool success = group->setNamedTarget(target);
                if (success)
                {
                    RCLCPP_INFO(this->get_logger(), "Moving to target: %s", target.c_str());
                    group->move();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Target %s not found", target.c_str());
                }
                // Wait for user input before proceeding to next step
                if (step_by_step){
                    std::cout << "Press Enter to continue to the next step..." << std::endl;
                    std::cin.get();
                }
            }
            RCLCPP_INFO(this->get_logger(), "Pick and place demo completed. Pressing Enter to repeat...");
            RCLCPP_INFO(this->get_logger(), "Or press Ctrl+C to exit.");
            std::cin.get();
        }
    }
private:
    bool step_by_step;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DemoPickPlaceNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // std::thread([&executor]() { executor.spin(); }).detach();
    std::thread spinner_thread([&executor]() { executor.spin(); });
    node->run_demo();
    rclcpp::shutdown();
    return 0;
}
