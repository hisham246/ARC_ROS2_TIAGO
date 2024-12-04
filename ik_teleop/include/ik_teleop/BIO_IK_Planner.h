#ifndef BIO_IK_PLANNER
#define BIO_IK_PLANNER

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ik_teleop/Utils.h>

#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <bio_ik/bio_ik.hpp>


class BIO_IK_Planner
{
public:
    public:
    BIO_IK_Planner(rclcpp::Node::SharedPtr Node);

    bool Collision_Callback(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* joint_group,
                            const double* joint_group_variable_values);
    
    bool ready();

    bool plan();
    
    void publish();

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    
    moveit::core::RobotModelConstPtr robot_model;

    double hz;

protected:
    // Nested class for group
    class BIO_IK_Group
    {
    public:
        BIO_IK_Group(BIO_IK_Planner& parent, std::string group_name, std::vector<std::string> joint_names, std::vector<double> current_position, std::string base_link, double hz, std::vector<double> joint_max_speed);

        void add_goals(std::vector<std::unique_ptr<bio_ik::Goal>>& goal_list);

        Eigen::Isometry3d get_goal();

        bool is_passive();
        
        void filter_state(moveit::core::RobotState &new_state, moveit::core::RobotStatePtr old_state);

        void publish_trajectory();

        void create_trajectory(std::vector<double> position);

        bool ready();

        bool new_message();

        void message_used();

        void commandCallback(const geometry_msgs::msg::PoseStamped& cmd);

        std::vector<double> interpolate_motion(std::vector<double> start_position, std::vector<double> end_position);

        std::string group_name_, goal_link_;
        
    protected:

        std::mutex group_mutex_;
        
        BIO_IK_Planner& parent_;

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
        bool initialized_=false, passive_group_=false, new_message_=false;
        
        double position_weight_, orientation_weight_, pose_weight_, freq_;
        

        tf2::Transform goal_pose_;

        trajectory_msgs::msg::JointTrajectory traj_;

        std::string base_link_;

        std::vector<std::string> joint_names_;
        std::vector<double> max_speeds_;
        
    };

    double center_weight_, min_disp_weight_, ik_timeout_;

    std::string main_group_, root_link_;
    std::vector<std::string> groups_;

    std::vector<BIO_IK_Group*> bio_groups_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::vector<double> previous_joints_;
    
    moveit::core::RobotStatePtr current_state_, goal_state_;

    moveit::core::GroupStateValidityCallbackFn callback_;
    
    rclcpp::Publisher<moveit_msgs::msg::RobotState>::SharedPtr demo_state_pub_, demo_ik_state_pub_;

    bool valid_goal_=false;
    
    rclcpp::Node::SharedPtr Node_;
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
};


#endif