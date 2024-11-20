#include <ik_teleop/PICK_IK_Planner.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    double hz;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("control_node", node_options);

    PICK_IK_Planner planner(move_group_node);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    
    rclcpp::Rate rate(planner.hz);
    rclcpp::Rate cpu_limitter(1000);

    while (rclcpp::ok())
    {
        if (planner.plan())
        {
            rate.sleep();
            planner.publish();
        }
        else
        {
            cpu_limitter.sleep();
        }
    }
    return 0;
}