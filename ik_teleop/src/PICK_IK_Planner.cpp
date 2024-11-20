#include <ik_teleop/PICK_IK_Planner.h>

PICK_IK_Planner::PICK_IK_Planner(rclcpp::Node::SharedPtr Node):
Node_(nullptr),
robot_model_loader_(nullptr),
current_state_(nullptr),
goal_state_(nullptr)
{
    Node_ = Node;
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(Node_);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(Node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    double default_max_speed;

    // Load parameters
    Node_->declare_parameter("frequency", 20.0);
    hz = Node_->get_parameter("frequency").as_double();

    Node_->declare_parameter("main_group", "movo_full");
    main_group_ = Node_->get_parameter("main_group").as_string();

    Node_->declare_parameter("root_link", "base_link");
    root_link_ = Node_->get_parameter("root_link").as_string();

    std::vector<std::string> empty;
    Node_->declare_parameter("groups", rclcpp::PARAMETER_STRING_ARRAY);
    groups_ = Node_->get_parameter("groups").as_string_array();

    Node_->declare_parameter("center_joints_weight", 0.01);
    center_weight_ = Node_->get_parameter("center_joints_weight").as_double();

    Node_->declare_parameter("min_displacement_weight", 0.2);
    min_disp_weight_ = Node_->get_parameter("min_displacement_weight").as_double();

    Node_->declare_parameter("ik_timeout", 0.2);
    ik_timeout_ = Node_->get_parameter("ik_timeout").as_double();

    Node_->declare_parameter("default_max_speed", 1.0);
    default_max_speed = Node_->get_parameter("default_max_speed").as_double();

    Node_->declare_parameter("global_solver", false);
    global_solver_ = Node_->get_parameter("global_solver").as_bool();
    
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

    if (!planning_scene_monitor_->getStateMonitor()->waitForCompleteState(main_group_, 10.0))
    {
      RCLCPP_ERROR_STREAM(Node_->get_logger(), "Timeout waiting for state");
      exit(-1);
    }

    callback_ = std::bind(&PICK_IK_Planner::Collision_Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // Wait for initialization
    current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

    moveit::core::RobotState temp(*planning_scene_monitor_->getStateMonitor()->getCurrentState());
    goal_state_ = std::make_shared<moveit::core::RobotState>(temp);

    current_state_->copyJointGroupPositions(main_group_, previous_joints_);

    robot_model = planning_scene_monitor_->getRobotModel();

    for (const std::string& group_name : groups_)
    {
        RCLCPP_ERROR_STREAM(Node_->get_logger(), group_name);
        const moveit::core::JointModelGroup *jmg = current_state_->getJointModelGroup(group_name);
        const std::vector<const moveit::core::JointModel::Bounds*>& bounds = jmg->getActiveJointModelsBounds();
        auto bij = jmg->getKinematicsSolverJointBijection();
        std::vector<std::string> joint_names = jmg->getActiveJointModelNames();

        std::vector<double> max_speeds;

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


        bio_groups_.push_back(new PICK_IK_Group(*this, group_name, joint_names, root_link_, hz, max_speeds));
        
        if (bio_groups_.back()->tip_frame_index > -1)
            tip_link_indices_.push_back(bio_groups_.back()->tip_frame_index);
    }

    robot = pick_ik::Robot::from(robot_model,
     current_state_->getJointModelGroup(main_group_), std::vector<size_t>());

    demo_state_pub_ = Node_->create_publisher<moveit_msgs::msg::RobotState>("display_robot_state", 1);

    RCLCPP_WARN_STREAM(Node_->get_logger(), "Bio IK planner ready to go!");

}

bool PICK_IK_Planner::ready()
{
    for (auto& group : bio_groups_)
    {
        if (!group->ready())
        {
            RCLCPP_WARN_STREAM_THROTTLE(Node_->get_logger(), *(Node_->get_clock()), 1.0, group->group_name_ + std::string("Not Ready Yet"));
            return false;
        }
    }
    return true;
}

bool PICK_IK_Planner::plan()
{
    if (!ready())
        return false;
    //current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

    bool new_message=true;
    for (auto& group : bio_groups_)
    {
        new_message=group->new_message();
        if (!new_message)
        {
            break;
        }
    }

    if (new_message)
    {
        
        kinematics::KinematicsQueryOptions query;
        query.return_approximate_solution = true;
        query.lock_redundant_joints = false;
        
        auto goals = std::vector<pick_ik::Goal>();
        auto pose_cost_fn_list = std::vector<pick_ik::PoseCostFn>();
        auto goal_frames = std::vector<Eigen::Isometry3d>();

        if (center_weight_ > 0.0) {
            goals.push_back(pick_ik::Goal{pick_ik::make_center_joints_cost_fn(robot), center_weight_});
        }
        // if (params.avoid_joint_limits_weight > 0.0) {
        //     goals.push_back(
        //         Goal{make_avoid_joint_limits_cost_fn(robot_), params.avoid_joint_limits_weight});
        // }
        if (min_disp_weight_ > 0.0) {
            goals.push_back(pick_ik::Goal{pick_ik::make_minimal_displacement_cost_fn(robot, previous_joints_),
                                 min_disp_weight_});
        }

        for (auto& group : bio_groups_)
        {
            group->add_goals(pose_cost_fn_list);
            goal_frames.push_back(group->get_goal());
        }

        auto const frame_tests =
            pick_ik::make_frame_tests(goal_frames, 0.001, 0.0001);

        auto const fk_fn = pick_ik::make_fk_fn(robot_model, current_state_->getJointModelGroup(main_group_),
            fk_mutex_, tip_link_indices_);

         // test if this is a valid solution
        auto const solution_fn =
            pick_ik::make_is_solution_test_fn(frame_tests, goals, 0.01, fk_fn);

        // single function used by gradient descent to calculate cost of solution
        auto const cost_fn = make_cost_fn(pose_cost_fn_list, goals, fk_fn);

        moveit::core::RobotState solution_state(*current_state_);

        std::vector<double> solution;

        solution_state.copyJointGroupPositions(main_group_, solution);

        // Set up initial optimization variables
        bool done_optimizing = false;
        bool found_valid_solution = false;
        double remaining_timeout = ik_timeout_;
        std::chrono::duration<double> total_optim_time{0.0};
        std::chrono::duration<double> const total_timeout{ik_timeout_};
        auto last_optim_time = std::chrono::system_clock::now();

        // If the initial state is not valid, restart from a random valid state.
        auto init_state = previous_joints_;
        if (!robot.is_valid_configuration(init_state)) {
            RCLCPP_WARN(
                Node_->get_logger(),
                "Initial guess exceeds joint limits. Regenerating a random valid configuration.");
            robot.set_random_valid_configuration(init_state);
        }

        // Optimize until a valid solution is found or we have timed out.
        while (!done_optimizing) {
            // Search for a solution using either the local or global solver.
            std::optional<std::vector<double>> maybe_solution;

            if (global_solver_) {
                pick_ik::MemeticIkParams ik_params;
                ik_params.population_size = static_cast<size_t>(16);
                ik_params.elite_size = static_cast<size_t>(4);
                ik_params.wipeout_fitness_tol = 0.0001;
                ik_params.stop_optimization_on_valid_solution =
                    false;
                ik_params.num_threads = static_cast<size_t>(1);
                ik_params.stop_on_first_soln = false;
                ik_params.max_generations = static_cast<int>(100);
                ik_params.max_time = remaining_timeout;

                ik_params.gd_params.step_size = 0.001;
                ik_params.gd_params.min_cost_delta = 1.0e-12;
                ik_params.gd_params.max_iterations = static_cast<int>(100);
                ik_params.gd_params.max_time = 0.005;

                maybe_solution = ik_memetic(previous_joints_,
                                            robot,
                                            cost_fn,
                                            solution_fn,
                                            ik_params,
                                            false,
                                            false /* No debug print */);
            }
            else
            {
                pick_ik::GradientIkParams gd_params;
                gd_params.step_size = 0.001;
                gd_params.min_cost_delta = 1.0e-12;
                gd_params.max_time = remaining_timeout;
                gd_params.max_iterations = static_cast<int>(100);
                gd_params.stop_optimization_on_valid_solution =
                    false;

                maybe_solution = ik_gradient(previous_joints_,
                                             robot,
                                             cost_fn,
                                             solution_fn,
                                             gd_params,
                                             false);
            }

            if (maybe_solution.has_value()) {
                // Set the output parameter solution.
                // Assumes that the angles were already wrapped by the solver.
                solution = maybe_solution.value();
            }
            else
            {
                solution = previous_joints_;
            }

            found_valid_solution = maybe_solution.has_value();

            // Check for timeout.
            auto const current_time = std::chrono::system_clock::now();
            total_optim_time += (current_time - last_optim_time);
            last_optim_time = current_time;
            bool const timeout_elapsed = (total_optim_time >= total_timeout);

            // If we found a valid solution or hit the timeout, we are done optimizing.
            // Otherwise, pick a random new initial seed and keep optimizing with the remaining
            // time.
            if (found_valid_solution || timeout_elapsed)
            {
                done_optimizing = true;
                solution_state.setJointGroupPositions(main_group_, solution);
            }
            else
            {
                robot.set_random_valid_configuration(init_state);
                remaining_timeout = ik_timeout_ - total_optim_time.count();
            }
        }
        
        if (!found_valid_solution)
        {
            RCLCPP_ERROR_STREAM_THROTTLE(Node_->get_logger(), *(Node_->get_clock()), 0.5, "No IK Found");
        }

        planning_scene_monitor::LockedPlanningSceneRO scene_ro(planning_scene_monitor_);
        if (!found_valid_solution || scene_ro->isStateColliding(solution_state, main_group_, true))
        {
            found_valid_solution = false;
            RCLCPP_ERROR_STREAM_THROTTLE(Node_->get_logger(), *(Node_->get_clock()), 0.5, "Planned End State Colliding");
        }

        if (found_valid_solution)
        {

            valid_goal_ = true;
            // *goal_state_ = solution_state;
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
        
        for (auto& group : bio_groups_)
        {
            std::vector<double> joint_state, start_state, interp_joint_state;
            
            goal_state_->copyJointGroupPositions(group->group_name_, joint_state);
            current_state_->copyJointGroupPositions(group->group_name_, start_state);

            interp_joint_state = group->interpolate_motion(start_state, joint_state);

            interp_state.setJointGroupPositions(group->group_name_, interp_joint_state);
        }

        std::vector<double> joint_state;

        if (scene_ro->isStateColliding(interp_state, main_group_))
        {
            RCLCPP_ERROR_STREAM(Node_->get_logger(), "Motion is in collision");
            return false;
        }

        for (auto& group : bio_groups_)
        {
            std::vector<double> joints;
            interp_state.copyJointGroupPositions(group->group_name_, joints);
            group->create_trajectory(joints);
        }
        
        current_state_->setJointGroupPositions(main_group_, joint_state); // = interp_state;
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
    
void PICK_IK_Planner::publish()
{
    if (!ready())
        return;
    for (auto& group : bio_groups_)
    {
        group->publish_trajectory();
    }
}


bool PICK_IK_Planner::Collision_Callback(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* joint_group,
                            const double* joint_group_variable_values)
{
    planning_scene_monitor::LockedPlanningSceneRO scene_ro(planning_scene_monitor_);
    robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
    robot_state->update();
    return !scene_ro->isStateColliding(*robot_state, joint_group->getName());
}


PICK_IK_Planner::PICK_IK_Group::PICK_IK_Group(PICK_IK_Planner& parent, std::string group_name,
    std::vector<std::string> joint_names, std::string base_link, double hz, std::vector<double> joint_max_speed)
: 
parent_(parent),
group_name_(group_name),
base_link_(base_link),
max_speeds_(joint_max_speed),
joint_names_(joint_names),
freq_(hz)
{
    std::string sub_topic, pub_topic;
    parent_.Node_->declare_parameter(group_name_+".passive_group", false);
    passive_group_ = parent.Node_->get_parameter(group_name_+".passive_group").as_bool();

    if (!passive_group_)
    {
        parent.Node_->declare_parameter(group_name_+".position_weight", 0.0);
        position_weight_ = parent.Node_->get_parameter(group_name_+".position_weight").as_double();
        
        parent.Node_->declare_parameter(group_name_+".orient_weight", 0.0);
        orientation_weight_ = parent.Node_->get_parameter(group_name_+".orient_weight").as_double();
        
        parent.Node_->declare_parameter(group_name_+".pose_weight", 0.0);
        pose_weight_ = parent.Node_->get_parameter(group_name_+".pose_weight").as_double();
        
        parent.Node_->declare_parameter(group_name_+".goal_link", "base_link");
        goal_link_ = parent.Node_->get_parameter(group_name_+".goal_link").as_string();
        
        auto const* link_model = parent.robot_model->getLinkModel(goal_link_);
        if (!link_model)
        {
            RCLCPP_ERROR_STREAM(parent.Node_->get_logger(), "link not found: "<< goal_link_);
            exit(-1);
        }
        tip_frame_index = link_model->getLinkIndex();

        parent.Node_->declare_parameter(group_name_+".sub_topic", "cmd");
        sub_topic = parent.Node_->get_parameter(group_name_+".sub_topic").as_string();

        sub_ = parent.Node_->create_subscription<geometry_msgs::msg::PoseStamped>(sub_topic, 1,
            std::bind(&PICK_IK_Group::commandCallback, this, std::placeholders::_1));
    }
    
    parent.Node_->declare_parameter(group_name_+".pub_topic", "traj");
    pub_topic = parent.Node_->get_parameter(group_name_+".pub_topic").as_string();

    pub_ = parent.Node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_topic, 1);
    
    traj_.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint temp_point;
    temp_point.time_from_start = rclcpp::Duration::from_nanoseconds(rclcpp::Rate(freq_).period().count());
    traj_.points.push_back(temp_point);
}

void PICK_IK_Planner::PICK_IK_Group::add_goals(std::vector<pick_ik::PoseCostFn>& PoseCostFnList)
{
    if (passive_group_)
        return;
    
    std::scoped_lock(group_mutex_);
    if (pose_weight_)
        PoseCostFnList.push_back(pick_ik::make_pose_cost_fn(goal_pose_, tip_frame_index, pose_weight_, pose_weight_));
    else
    {
        PoseCostFnList.push_back(pick_ik::make_pose_cost_fn(goal_pose_, tip_frame_index, position_weight_, orientation_weight_));
    }
}

Eigen::Isometry3d PICK_IK_Planner::PICK_IK_Group::get_goal()
{
    std::scoped_lock(group_mutex_);
    return goal_pose_;
}


void PICK_IK_Planner::PICK_IK_Group::filter_state(moveit::core::RobotState &new_state, moveit::core::RobotStatePtr old_state)
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

    Eigen::Isometry3d goal_pose = new_state.getFrameTransform(base_link_).inverse()*goal_pose_;
    
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

void PICK_IK_Planner::PICK_IK_Group::create_trajectory(std::vector<double> position)
{
    // traj_.header.frame_id = base_link_;
    traj_.points[0].positions = position;
    traj_.points[0].time_from_start = rclcpp::Duration::from_seconds(1.0/freq_);
}

void PICK_IK_Planner::PICK_IK_Group::publish_trajectory()
{
    pub_->publish(traj_);
}

bool PICK_IK_Planner::PICK_IK_Group::ready()
{
    return passive_group_ || initialized_;
}

void PICK_IK_Planner::PICK_IK_Group::commandCallback(const geometry_msgs::msg::PoseStamped& cmd)
{
    std::scoped_lock(group_mutex_);
    
    try
    {
        geometry_msgs::msg::PoseStamped temp;
        temp.pose = cmd.pose;
        temp.header = cmd.header;
        tf2::convert(parent_.tf_buffer->transform(temp, base_link_, tf2::durationFromSec(0.1)).pose, goal_pose_);
        initialized_ = true;
        new_message_ = true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN_STREAM(parent_.Node_->get_logger(), "Could NOT transform: " << ex.what());
    }

}

std::vector<double> PICK_IK_Planner::PICK_IK_Group::interpolate_motion(std::vector<double> start, std::vector<double> end)
{
    double scaling = 1;
    for (int i = 0; i < end.size(); ++i)
    {
        if (max_speeds_[i] <= 0)
            continue;

        double temp_scale = std::abs(start[i] - end[i]) * freq_ / max_speeds_[i];

        if (temp_scale > scaling && !isinf(temp_scale))
        {
            scaling = temp_scale;
        }
    }

    std::vector<double> interp(end);

    if (scaling > 1.0)
    {
        for (int j = 0; j < interp.size(); ++j)
        {
            interp[j] = start[j] + (end[j] - start[j])/scaling;
        }
    }

    return interp;
}

bool PICK_IK_Planner::PICK_IK_Group::new_message()
{
    return new_message_ || passive_group_;
}

void PICK_IK_Planner::PICK_IK_Group::message_used()
{
    new_message_ = false;
}
