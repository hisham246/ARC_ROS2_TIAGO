#include <ik_teleop/BIO_IK_Planner.h>

BIO_IK_Planner::BIO_IK_Planner(rclcpp::Node::SharedPtr Node):
    Node_(nullptr),
    robot_model_loader_(nullptr),
    current_state_(nullptr),
    goal_state_(nullptr)
{
    Node_ = Node;
    RCLCPP_WARN_STREAM(Node_->get_logger(), "Starting Planner");
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(Node_);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(Node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    double default_max_speed;

    // Load parameters
    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load Frequency");
    hz = Node_->get_parameter("frequency").as_double();

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load main group");
    main_group_ = Node_->get_parameter("main_group").as_string();

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load root link");
    root_link_ = Node_->get_parameter("root_link").as_string();

    std::vector<std::string> empty;
    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load groups");
    groups_ = Node_->get_parameter("groups").as_string_array();

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load center joint weights");
    center_weight_ = Node_->get_parameter("center_joints_weight").as_double();

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load min disp weight");
    min_disp_weight_ = Node_->get_parameter("min_displacement_weight").as_double();
    
    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load min disp joints");
    min_displacement_joints_ = Node_->get_parameter("min_displacement_joints").as_string_array();
    
    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load min disp joints weights");
    min_displacement_joints_weights_= Node_->get_parameter("min_displacement_joints_weights").as_double_array();

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load ik timeout");
    ik_timeout_ = Node_->get_parameter("ik_timeout").as_double();

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Load max speed");
    default_max_speed = Node_->get_parameter("default_max_speed").as_double();
    
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(Node_, robot_model_loader_, "planning_scene_monitor");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR_STREAM(Node_->get_logger(), "PlanningSceneMonitor setup failed.");
      exit(-1);
    }

    // Start the planning scene monitor
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        true);
    planning_scene_monitor_->startStateMonitor();

    if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(main_group_, 100.0))
    {
      RCLCPP_ERROR_STREAM(Node_->get_logger(), "Timeout waiting for state");
      exit(-1);
    }

    callback_ = std::bind(&BIO_IK_Planner::Collision_Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // Wait for initialization
    current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

    moveit::core::RobotState temp(*planning_scene_monitor_->getStateMonitor()->getCurrentState());
    goal_state_ = std::make_shared<moveit::core::RobotState>(temp);

    current_state_->copyJointGroupPositions(main_group_, previous_joints_);
    // RCLCPP_WARN_STREAM(Node_->get_logger(), "Joint Positions");
    // for (const double& val : previous_joints_)
    // {
    //     RCLCPP_WARN_STREAM(Node_->get_logger(), val);
    // }

    robot_model = planning_scene_monitor_->getRobotModel();

    const moveit::core::JointModelGroup *jmg = current_state_->getJointModelGroup(main_group_);

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Potential Root Links \n" << robot_model->getRootLink()->getName() << "\n" <<
        jmg->getCommonRoot()->getParentLinkModel()->getName() << "\n" << root_link_);
    
    root_link_ = robot_model->getRootLink()->getName();
    
    // root_link_ = jmg->getCommonRoot()->getParentLinkModel()->getName();
    const std::vector<std::string>& joint_names = jmg->getVariableNames();
    RCLCPP_WARN_STREAM(Node_->get_logger(), "joints");
    for (const std::string& name : joint_names)
    {
        RCLCPP_WARN_STREAM(Node_->get_logger(), name);
    }
    
    for (const std::string& group_name : groups_)
    {
        RCLCPP_WARN_STREAM(Node_->get_logger(), "Making group: " << group_name);
        const moveit::core::JointModelGroup *jmg = current_state_->getJointModelGroup(group_name);
        const std::vector<const moveit::core::JointModel::Bounds*>& bounds = jmg->getActiveJointModelsBounds();
        auto bij = jmg->getKinematicsSolverJointBijection();
        std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
        // for (auto name : joint_names)
        // {
        //     RCLCPP_WARN_STREAM(Node_->get_logger(), name);
        // }

        std::vector<double> max_speeds, current_position;

        for (std::size_t j = 0; j < joint_names.size(); ++j)
        {
            if (j < bij.size())
            {
                const std::vector<moveit::core::VariableBounds>* var_bounds = bounds[bij[j]];
                if (var_bounds->size() != 1)
                {
                    max_speeds.push_back(default_max_speed);
                }
                else
                {
                    max_speeds.push_back((*var_bounds)[0].max_velocity_);
                }
            }
            else
            {
                max_speeds.push_back(default_max_speed);
            }
        }

        current_state_->copyJointGroupPositions(group_name, current_position);

        bio_groups_.push_back(new BIO_IK_Group(*this, group_name, joint_names, current_position, root_link_, hz, max_speeds));
    }
    
    demo_ik_state_pub_ = Node_->create_publisher<moveit_msgs::msg::RobotState>("display_ik_robot_state", 1);
    demo_state_pub_ = Node_->create_publisher<moveit_msgs::msg::RobotState>("display_robot_state", 1);

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Bio IK planner ready to go!");

}

bool BIO_IK_Planner::ready()
{
    for (auto& group : bio_groups_)
    {
        if (!group->ready())
        {
            auto& clock = *(Node_->get_clock());
            RCLCPP_WARN_STREAM_THROTTLE(Node_->get_logger(), clock, 5.0, group->group_name_ + std::string(" Not Ready Yet"));
            return false;
        }
    }
    return true;
}

bool BIO_IK_Planner::plan()
{
    if (!ready())
        return false;
    //current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

    bool new_message=true;
    for (auto& group : bio_groups_)
    {
        new_message&=group->new_message();
        if (!new_message)
        {
            break;
        }
    }

    if (new_message)
    {

        bio_ik::BioIKKinematicsQueryOptions query;
        query.return_approximate_solution = true;
        query.lock_redundant_joints = false;

        if (center_weight_)
            query.goals.emplace_back(new bio_ik::CenterJointsGoal(center_weight_, true));
        if (min_disp_weight_)
            query.goals.emplace_back(new bio_ik::MinimalDisplacementGoal(min_disp_weight_, true));
        
        if (min_displacement_joints_.size())
        {
            for (int i=0; i < std::min(min_displacement_joints_weights_.size(),min_displacement_joints_.size()); i++)
            {
                query.goals.emplace_back(new MinimizeJointVariableGoal(min_displacement_joints_[i], min_displacement_joints_weights_[i]));
            }
        }
        
        EigenSTL::vector_Isometry3d poses;
        std::vector<std::string> goal_tips;

        for (auto& group : bio_groups_)
        {
            group->add_goals(query.goals);
            if (!group->is_passive())
            {
                // poses.push_back(group->get_goal());
                // goal_tips.push_back(group->goal_link_);
            }
        }
        

        moveit::core::RobotState solution_state(*current_state_);
        
        bool found_ik = solution_state.setFromIK(solution_state.getJointModelGroup(main_group_),
                                                poses,
                                                goal_tips,
                                                ik_timeout_,
                                                moveit::core::GroupStateValidityCallbackFn(), // callback_,
                                                query);
        
        // RCLCPP_ERROR_STREAM(Node_->get_logger(), "Solution Fitness " << query.solution_fitness);

        if (!found_ik)
        {
            RCLCPP_WARN_STREAM_THROTTLE(Node_->get_logger(), *(Node_->get_clock()), 0.5, "No IK Found");
        }
        else
        {
            moveit_msgs::msg::RobotState display_state;

            moveit::core::robotStateToRobotStateMsg(solution_state, display_state);

            demo_ik_state_pub_->publish(display_state);
            // std::cout << "Current States" << std::endl;
            // current_state_->printStatePositions(std::cout);
            // std::cout << "Solutionn States" << std::endl;
            // solution_state.printStatePositions(std::cout);
            planning_scene_monitor::LockedPlanningSceneRO scene_ro(planning_scene_monitor_);
            if (scene_ro->isStateColliding(solution_state, main_group_, true))
            {
                found_ik = false;
                RCLCPP_WARN_STREAM_THROTTLE(Node_->get_logger(), *(Node_->get_clock()), 0.5, "Planned End State Colliding");
            }
        }

        if (found_ik)
        {

            valid_goal_ = true;
            *goal_state_ = solution_state;
            for (auto& group : bio_groups_)
            {
                group->filter_state(solution_state, goal_state_);
            }
        }
    }
    

    if (valid_goal_)
    {
        planning_scene_monitor::LockedPlanningSceneRO scene_ro(planning_scene_monitor_);
        moveit::core::RobotState interp_state(*current_state_);
        
        std::vector<double> joint_state, start_state, interp_joint_state;

        for (auto& group : bio_groups_)
        {
            
            goal_state_->copyJointGroupPositions(group->group_name_, joint_state);
            // current_state_->setJointGroupPositions(group->group_name_, joint_state);

            current_state_->copyJointGroupPositions(group->group_name_, start_state);

            interp_joint_state = group->interpolate_motion(start_state, joint_state);

            interp_state.setJointGroupPositions(group->group_name_, interp_joint_state);
        }

        if (scene_ro->isStateColliding(interp_state, main_group_))
        {
            RCLCPP_ERROR_STREAM(Node_->get_logger(), "Motion is in collision");
            return false;
        }

        for (auto& group : bio_groups_)
        {
            std::vector<double> joints;
            interp_state.copyJointGroupPositions(group->group_name_, joints);
            // current_state_->copyJointGroupPositions(group->group_name_, joints);
            group->create_trajectory(joints);
        }
        
        // interp_state.copyJointGroupPositions(main_group_, interp_joint_state);

        // current_state_->setJointGroupPositions(main_group_, interp_joint_state); // = interp_state;
        *current_state_ = interp_state;
        
        current_state_->copyJointGroupPositions(main_group_, previous_joints_);

        moveit_msgs::msg::RobotState display_state;

        moveit::core::robotStateToRobotStateMsg(*current_state_, display_state);

        demo_state_pub_->publish(display_state);
        return true;
    }
    else
    {
        return false;
    }

}
    
void BIO_IK_Planner::publish()
{
    if (!ready())
        return;
    for (auto& group : bio_groups_)
    {
        group->publish_trajectory();
    }
}


bool BIO_IK_Planner::Collision_Callback(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* joint_group,
                            const double* joint_group_variable_values)
{
    // RCLCPP_WARN_STREAM(Node_->get_logger(), "collision callback");
    planning_scene_monitor::LockedPlanningSceneRO scene_ro(planning_scene_monitor_);
    robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
    robot_state->update();
    return !scene_ro->isStateColliding(*robot_state, joint_group->getName());
}


BIO_IK_Planner::BIO_IK_Group::BIO_IK_Group(BIO_IK_Planner& parent, std::string group_name,
    std::vector<std::string> joint_names, std::vector<double> current_position, std::string base_link, double hz, std::vector<double> joint_max_speed)
: 
parent_(parent),
group_name_(group_name),
base_link_(base_link),
max_speeds_(joint_max_speed),
joint_names_(joint_names),
freq_(hz)
{
    std::string sub_topic, pub_topic;
    RCLCPP_WARN_STREAM(parent_.Node_->get_logger(), "Load group params");
    passive_group_ = parent.Node_->get_parameter(group_name_+".passive_group").as_bool();

    if (!passive_group_)
    {
        position_weight_ = parent.Node_->get_parameter(group_name_+".position_weight").as_double();
        
        orientation_weight_ = parent.Node_->get_parameter(group_name_+".orient_weight").as_double();
        
        pose_weight_ = parent.Node_->get_parameter(group_name_+".pose_weight").as_double();
        
        z_weight_ = parent.Node_->get_parameter(group_name_+".z_weight").as_double();

        goal_link_ = parent.Node_->get_parameter(group_name_+".goal_link").as_string();
        
        auto const* link_model = parent.robot_model->getLinkModel(goal_link_);
        if (!link_model)
        {
            RCLCPP_ERROR_STREAM(parent.Node_->get_logger(), "link not found: "<< goal_link_);
            exit(-1);
        }

        sub_topic = parent.Node_->get_parameter(group_name_+".sub_topic").as_string();

        sub_ = parent.Node_->create_subscription<geometry_msgs::msg::PoseStamped>(sub_topic, 1,
            std::bind(&BIO_IK_Group::commandCallback, this, std::placeholders::_1));
    }
    
    pub_topic = parent.Node_->get_parameter(group_name_+".pub_topic").as_string();

    pub_ = parent.Node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_topic, 1);
    
    traj_.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint temp_point;
    temp_point.positions = current_position;
    temp_point.time_from_start = rclcpp::Duration::from_nanoseconds(rclcpp::Rate(freq_).period().count());
    traj_.points.push_back(temp_point);
}

void BIO_IK_Planner::BIO_IK_Group::add_goals(std::vector<std::unique_ptr<bio_ik::Goal>>& goal_list)
{
    if (passive_group_)
        return;
    
    group_mutex_.lock();

    if (pose_weight_)
        goal_list.emplace_back(new bio_ik::PoseGoal(goal_link_, goal_pose_.getOrigin(), goal_pose_.getRotation(), pose_weight_));
    if (position_weight_)
        goal_list.emplace_back(new bio_ik::PositionGoal(goal_link_, goal_pose_.getOrigin(), position_weight_));
    if (orientation_weight_)
        goal_list.emplace_back(new bio_ik::OrientationGoal(goal_link_, goal_pose_.getRotation(), orientation_weight_));
    if (z_weight_)
        goal_list.emplace_back(new PositionZGoal(goal_link_, goal_pose_.getOrigin(), z_weight_));
    
    group_mutex_.unlock();
}

Eigen::Isometry3d BIO_IK_Planner::BIO_IK_Group::get_goal()
{
    Eigen::Isometry3d pose;
    if (passive_group_)
        return pose;
    
    std::scoped_lock(group_mutex_);
    pose = tf2::transformToEigen(tf2::toMsg(goal_pose_));
    return pose;
}

bool BIO_IK_Planner::BIO_IK_Group::is_passive()
{
    return passive_group_;
}


void BIO_IK_Planner::BIO_IK_Group::filter_state(moveit::core::RobotState &new_state, moveit::core::RobotStatePtr old_state)
{
    std::vector<double> new_angles, old_angles;
    new_state.copyJointGroupPositions(group_name_, new_angles);
    old_state->copyJointGroupPositions(group_name_, old_angles);

    if (passive_group_)
    {
        old_state->setJointGroupActivePositions(group_name_, new_angles);
        return;
    }
    
    Eigen::Isometry3d new_pose = new_state.getFrameTransform(goal_link_);
    Eigen::Isometry3d old_pose = old_state->getFrameTransform(goal_link_);

    Eigen::Isometry3d goal_pose = new_state.getFrameTransform(base_link_).inverse()*tf2::transformToEigen(tf2::toMsg(goal_pose_));
    
    double angular_movement = teleop_utils::get_angular_movement(new_angles, old_angles);
    double linear_improvement =  teleop_utils::get_linear_distance(old_pose, goal_pose) - teleop_utils::get_linear_distance(new_pose, goal_pose);
    double angular_improvement =  teleop_utils::get_angular_distance(old_pose, goal_pose) - teleop_utils::get_angular_distance(new_pose, goal_pose);

    if (((position_weight_ != 0 || pose_weight_ != 0) && (linear_improvement > 0 || std::fabs(angular_improvement) >= 0.2)) || 
        (position_weight_ == 0 && pose_weight_ == 0 && angular_improvement > 0)) // &&
        //  angular_movement/double(new_angles.size())/double(1e3) < (linear_improvement + angular_improvement/100.0))
    {
        old_state->setJointGroupActivePositions(group_name_, new_angles);
    }
    // else
    // {
    //     ROS_WARN_STREAM_THROTTLE(0.5, group_name_ << ": has been filtered");
    // }
}

void BIO_IK_Planner::BIO_IK_Group::create_trajectory(std::vector<double> position)
{
    std::vector<double> velocity;
    for (int i = 0; i < position.size(); i++)
    {
        velocity.push_back((position[i] - traj_.points[0].positions[i])*freq_);
    }
    traj_.header.frame_id = base_link_;
    traj_.points[0].positions = position;
    
    traj_.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0/freq_);
}

void BIO_IK_Planner::BIO_IK_Group::publish_trajectory()
{
    pub_->publish(traj_);
}

bool BIO_IK_Planner::BIO_IK_Group::ready()
{
    return passive_group_ || initialized_;
}

void BIO_IK_Planner::BIO_IK_Group::commandCallback(const geometry_msgs::msg::PoseStamped& cmd)
{
    std::scoped_lock(group_mutex_);
    
    try
    {
        // RCLCPP_WARN_STREAM(parent_.Node_->get_logger(), group_name_ << ": " << (double)temp.pose.position.x << " " << 
        //     (double)temp.pose.position.y << " " << (double)temp.pose.position.z);
        tf2::convert(parent_.tf_buffer->transform(cmd, base_link_, tf2::durationFromSec(0.1)).pose, goal_pose_);
        // RCLCPP_WARN_STREAM(parent_.Node_->get_logger(), group_name_ << ": " << (double)goal_pose_.translation().x() << " " << 
        //     (double)goal_pose_.translation().y() << " " << (double)goal_pose_.translation().z());
        initialized_ = true;
        new_message_ = true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN_STREAM(parent_.Node_->get_logger(), "Could NOT transform: " << ex.what());
    }

}

std::vector<double> BIO_IK_Planner::BIO_IK_Group::interpolate_motion(std::vector<double> start, std::vector<double> end)
{
    double scaling = 1.0;
    for (int i = 0; i < end.size(); ++i)
    {
        if (max_speeds_[i] <= 0.01)
            continue;

        double temp_scale = std::abs(start[i] - end[i]) * freq_ / std::abs(max_speeds_[i]);

        if (temp_scale > scaling && !isinf(temp_scale))
        {
            scaling = temp_scale;
        }
    }

    // RCLCPP_WARN_STREAM(parent_.Node_->get_logger(), "Scaling for group: " << group_name_ << " is " << scaling);
    

    std::vector<double> interp;

    if (scaling > 1.0)
    {
        for (int j = 0; j < start.size(); ++j)
        {
            interp.push_back(start[j] + ((end[j] - start[j])/scaling));
        }
        return interp;
    }

    return end;
}

bool BIO_IK_Planner::BIO_IK_Group::new_message()
{
    return new_message_ || passive_group_;
}

void BIO_IK_Planner::BIO_IK_Group::message_used()
{
    new_message_ = false;
}
