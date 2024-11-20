#ifndef PICK_IK_PLANNER
#define PICK_IK_PLANNER

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ik_teleop/PICK_IK_Planner.h>
#include <ik_teleop/Utils.h>

#include <pick_ik/forward_kinematics.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class PICK_IK_Planner
{
public:
    public:
    PICK_IK_Planner(rclcpp::Node::SharedPtr Node);

    bool Collision_Callback(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* joint_group,
                            const double* joint_group_variable_values);
    
    bool ready();

    bool plan();
    
    void publish();

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    pick_ik::Robot robot;
    
    moveit::core::RobotModelConstPtr robot_model;

    double hz;

protected:
    // Nested class for group
    class PICK_IK_Group
    {
    public:
        PICK_IK_Group(PICK_IK_Planner& parent, std::string group_name, std::vector<std::string> joint_names, std::string base_link, double hz, std::vector<double> joint_max_speed);

        void add_goals(std::vector<pick_ik::PoseCostFn>& PoseCostFnList);

        Eigen::Isometry3d get_goal();

        void filter_state(moveit::core::RobotState &new_state, moveit::core::RobotStatePtr old_state);

        void publish_trajectory();

        void create_trajectory(std::vector<double> position);

        bool ready();

        bool new_message();

        void message_used();

        void commandCallback(const geometry_msgs::msg::PoseStamped& cmd);

        std::vector<double> interpolate_motion(std::vector<double> start_position, std::vector<double> end_position);

        std::string group_name_;
        size_t tip_frame_index = -1;

    protected:

        std::mutex group_mutex_;
        
        PICK_IK_Planner& parent_;

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
        bool initialized_=false, passive_group_=false, new_message_=false;
        
        double position_weight_, orientation_weight_, pose_weight_, freq_;
        

        Eigen::Isometry3d goal_pose_;

        trajectory_msgs::msg::JointTrajectory traj_;

        std::string goal_link_, base_link_;

        std::vector<std::string> joint_names_;
        std::vector<double> max_speeds_;
        
    };

    double center_weight_, min_disp_weight_, ik_timeout_;

    mutable std::mutex fk_mutex_;

    std::string main_group_, root_link_;
    std::vector<std::string> groups_;
    std::vector<size_t> tip_link_indices_;

    std::vector<PICK_IK_Group*> bio_groups_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::vector<double> previous_joints_;
    
    moveit::core::RobotStatePtr current_state_, goal_state_;

    moveit::core::GroupStateValidityCallbackFn callback_;
    
    rclcpp::Publisher<moveit_msgs::msg::RobotState>::SharedPtr demo_state_pub_;

    bool valid_goal_=false, global_solver_;
    
    rclcpp::Node::SharedPtr Node_;
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
};


#endif