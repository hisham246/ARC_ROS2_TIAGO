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
#include <sensor_msgs/msg/joint_state.hpp>

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
    
    bool plan();
    
    void publish();

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    
    moveit::core::RobotModelConstPtr robot_model;
    
    moveit::core::RobotStatePtr current_state_, goal_state_;

    double hz;

protected:
    // Nested class for additional pose goal
    class BIO_IK_Pose_Goal
    {
    public:
        BIO_IK_Pose_Goal(BIO_IK_Planner& parent, std::string name, std::string base_link, bool passive);

        void add_goals(std::vector<std::unique_ptr<bio_ik::Goal>>& goal_list);

        Eigen::Isometry3d get_goal();

        bool is_passive();

        bool new_message();

        void message_used();

        void commandCallback(const geometry_msgs::msg::PoseStamped& cmd);

        std::string goal_link_;
        
    protected:

        std::mutex group_mutex_;
        
        BIO_IK_Planner& parent_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
        bool initialized_=false, new_message_=false, passive_=false;
        
        double position_weight_, orientation_weight_, pose_weight_, z_weight_, freq_;
        
        tf2::Transform goal_pose_;

        std::string base_link_;
    };

    // Nested class for group - Inherited from Pose Goal
    class BIO_IK_Group : public BIO_IK_Pose_Goal
    {
    public:
        BIO_IK_Group(BIO_IK_Planner& parent, std::string group_name, std::vector<std::string> joint_names,
            std::vector<double> current_position, std::string base_link, double hz,
            std::vector<double> joint_max_speed,bool passive);
        
        void filter_state(moveit::core::RobotState &new_state, moveit::core::RobotStatePtr old_state);

        void publish_trajectory();

        void create_trajectory(std::vector<double> position);

        std::vector<double> interpolate_motion(std::vector<double> start_position, std::vector<double> end_position);

        std::string group_name_;
        
    protected:
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;

        trajectory_msgs::msg::JointTrajectory traj_;
        
        std::vector<std::string> joint_names_;
        std::vector<double> max_speeds_;
        
        double freq_;
    };

    // Nested class for additional joint goal
    class BIO_IK_Joint_Goal
    {
    public:
        BIO_IK_Joint_Goal(BIO_IK_Planner& parent, std::string name);

        void add_goals(std::vector<std::unique_ptr<bio_ik::Goal>>& goal_list);

        bool new_message();

        void message_used();

        void commandCallback(const sensor_msgs::msg::JointState& cmd);
        
    protected:
        std::mutex group_mutex_;

        BIO_IK_Planner& parent_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
        bool initialized_=false, new_message_=false;
        
        double default_weight_;
        
        sensor_msgs::msg::JointState goal_;
        
    };

    double center_weight_, min_disp_weight_, ik_timeout_;

    std::string main_group_, root_link_;
    std::vector<std::string> groups_, min_displacement_joints_;

    std::vector<BIO_IK_Group*> bio_groups_;
    std::vector<BIO_IK_Pose_Goal*> bio_pose_goals_;
    std::vector<BIO_IK_Joint_Goal*> bio_joint_goals_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::vector<double> previous_joints_, min_displacement_joints_weights_;
    
    moveit::core::GroupStateValidityCallbackFn callback_;
    
    rclcpp::Publisher<moveit_msgs::msg::RobotState>::SharedPtr demo_state_pub_, demo_ik_state_pub_;

    bool valid_goal_=false;
    
    rclcpp::Node::SharedPtr Node_;
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
};


#endif