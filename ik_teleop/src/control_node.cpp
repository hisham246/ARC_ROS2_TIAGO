#include <ik_teleop/BIO_IK_Planner.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("control_node", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread ros_thread([&executor]() { executor.spin(); });
    
    BIO_IK_Planner planner(node);

    rclcpp::Rate rate(planner.hz);
    rclcpp::Rate cpu_limitter(1000);

    while (rclcpp::ok())
    {
        if (planner.plan())
        {
            planner.publish();
            rate.sleep();
        }
        else
        {
            cpu_limitter.sleep();
        }
        // cpu_limitter.sleep();
        // RCLCPP_WARN(node->get_logger(), "Hi :)");
    }
    ros_thread.detach();
    return 0;
}