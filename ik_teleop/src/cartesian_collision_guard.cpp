// Collision guard for Cartesian impedance teleop.
//
// Sits between the VR node and the Cartesian impedance controller:
//   input_topic  (geometry_msgs/Pose, root_link frame): the wanted end effector pose
//   output_topic (geometry_msgs/Pose, root_link frame): the same pose, pulled back towards the measured
//                end effector pose when moving towards it would bring the arm closer than `margin`
//                to the rest of the robot (or to world objects in the planning scene).
//
// The check works on the MEASURED joint state, because the impedance controller (not this node) chooses the
// joint configuration. The gap to the wanted pose is mapped to a joint step with a damped pseudo-inverse of the
// arm Jacobian, and the states q + s*dq are tested for s = 1, 0.75, 0.5, 0.25. The largest safe s wins.
// Same ideas as the BIO IK planner (end state check, motion check, keep the last safe target), applied
// in front of the controller. The controller itself is not modified.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

namespace
{
struct Clearance
{
    double distance = std::numeric_limits<double>::max();
    std::string link_a, link_b;
};

Eigen::Isometry3d to_isometry(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (q.norm() < 1e-9)
        q = Eigen::Quaterniond::Identity();
    q.normalize();
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = q.toRotationMatrix();
    t.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return t;
}

geometry_msgs::msg::Pose to_pose(const Eigen::Isometry3d& t)
{
    geometry_msgs::msg::Pose pose;
    Eigen::Quaterniond q(t.linear());
    q.normalize();
    pose.position.x = t.translation().x();
    pose.position.y = t.translation().y();
    pose.position.z = t.translation().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}
}  // namespace

class CollisionGuard
{
public:
    explicit CollisionGuard(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        group_ = param<std::string>("group", "arm_right");
        ee_link_ = param<std::string>("ee_link", "arm_right_7_link");
        root_link_ = param<std::string>("root_link", "torso_lift_link");
        const std::string input_topic = param<std::string>("input_topic", "/vr_cartesian/desired_pose");
        const std::string output_topic =
            param<std::string>("output_topic", "/cartesian_impedance_controller_right/desired_pose");
        margin_ = param<double>("margin", 0.03);                     // required clearance [m]
        lookahead_pos_ = param<double>("lookahead_position", 0.05);  // part of the gap that is checked [m]
        lookahead_rot_ = param<double>("lookahead_rotation", 0.3);   // [rad]
        damping_ = param<double>("damping", 0.05);
        max_joint_step_ = param<double>("max_joint_step", 0.2);      // limit of the linearised step [rad]
        recover_per_tick_ = param<double>("recover_per_tick", 0.1);  // how fast the scale grows back
        input_timeout_ = param<double>("input_timeout", 0.3);        // [s]
        const double rate = param<double>("rate", 50.0);

        RCLCPP_INFO(node_->get_logger(), "Loading the robot model");
        robot_model_loader::RobotModelLoader::Options options("robot_description");
        options.load_kinematics_solvers_ = false;
        loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, options);
        monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, loader_, "guard_planning_scene_monitor");
        if (!monitor_->getPlanningScene())
        {
            RCLCPP_FATAL(node_->get_logger(), "PlanningSceneMonitor setup failed");
            std::exit(1);
        }
        monitor_->startSceneMonitor();
        monitor_->startWorldGeometryMonitor(
            planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
            planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, true);
        monitor_->startStateMonitor();

        model_ = monitor_->getRobotModel();
        jmg_ = model_->getJointModelGroup(group_);
        ee_model_ = model_->getLinkModel(ee_link_);
        if (!jmg_ || !ee_model_ || !model_->getLinkModel(root_link_))
        {
            RCLCPP_FATAL(node_->get_logger(), "Unknown group '%s', ee_link '%s' or root_link '%s'", group_.c_str(),
                         ee_link_.c_str(), root_link_.c_str());
            std::exit(1);
        }
        if (!monitor_->getStateMonitor()->waitForCompleteState(group_, 60.0))
        {
            RCLCPP_FATAL(node_->get_logger(), "Timeout waiting for the joint states of '%s'", group_.c_str());
            std::exit(1);
        }

        pub_ = node_->create_publisher<geometry_msgs::msg::Pose>(output_topic, 1);
        sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            input_topic, 10, [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                target_ = *msg;
                last_input_ = node_->now();
                have_input_ = true;
            });
        timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
        RCLCPP_INFO(node_->get_logger(), "Collision guard ready: %s -> %s, group %s, margin %.3f m", input_topic.c_str(),
                    output_topic.c_str(), group_.c_str(), margin_);
    }

private:
    template <typename T>
    T param(const std::string& name, const T& default_value)
    {
        if (!node_->has_parameter(name))
            node_->declare_parameter(name, default_value);
        return node_->get_parameter(name).get_value<T>();
    }

    // Smallest distance between the group's links and the rest of the robot / the world objects
    Clearance clearance(const moveit::core::RobotState& state)
    {
        planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
        collision_detection::DistanceRequest request;
        request.type = collision_detection::DistanceRequestType::GLOBAL;
        request.enable_signed_distance = true;
        request.distance_threshold = 4.0 * margin_;
        request.acm = &scene->getAllowedCollisionMatrix();
        request.group_name = group_;
        request.enableGroup(model_);

        Clearance best;
        auto consider = [&best](const collision_detection::DistanceResult& result) {
            if (result.minimum_distance.distance < best.distance)
            {
                best.distance = result.minimum_distance.distance;
                best.link_a = result.minimum_distance.link_names[0];
                best.link_b = result.minimum_distance.link_names[1];
            }
        };

        collision_detection::DistanceResult self_result;
        scene->getCollisionEnv()->distanceSelf(request, self_result, state);
        consider(self_result);

        if (scene->getWorld()->size() > 0)
        {
            collision_detection::DistanceResult world_result;
            scene->getCollisionEnv()->distanceRobot(request, world_result, state);
            consider(world_result);
        }
        return best;
    }

    void tick()
    {
        const rclcpp::Time now = node_->now();
        if (!have_input_ || (now - last_input_).seconds() > input_timeout_)
        {
            scale_ = 1.0;
            return;  // the VR node is disabled or silent: send nothing, the controller holds its last pose
        }
        if (!monitor_->getStateMonitor()->haveCompleteState(rclcpp::Duration::from_seconds(0.5)))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Joint states are stale, not forwarding the target");
            return;
        }

        moveit::core::RobotState state(*monitor_->getStateMonitor()->getCurrentState());
        state.updateLinkTransforms();

        const Eigen::Isometry3d root_in_model = state.getGlobalLinkTransform(root_link_);
        const Eigen::Isometry3d current = root_in_model.inverse() * state.getGlobalLinkTransform(ee_link_);
        const Eigen::Isometry3d target = to_isometry(target_);

        // gap to the wanted pose, in the root frame
        const Eigen::Vector3d dp = target.translation() - current.translation();
        const Eigen::AngleAxisd dr(target.linear() * current.linear().transpose());
        const Eigen::Vector3d dw = dr.axis() * dr.angle();

        double shrink = 1.0;
        if (dp.norm() > lookahead_pos_)
            shrink = std::min(shrink, lookahead_pos_ / dp.norm());
        if (dw.norm() > lookahead_rot_)
            shrink = std::min(shrink, lookahead_rot_ / dw.norm());

        double wanted_scale = 1.0;
        Clearance blocker;
        if (dp.norm() > 1e-4 || dw.norm() > 1e-3)
            wanted_scale = safe_scale(state, root_in_model, shrink * dp, shrink * dw, blocker);

        // shrink at once, recover slowly, so the output does not chatter at the edge of the margin
        scale_ = (wanted_scale < scale_) ? wanted_scale : std::min(wanted_scale, scale_ + recover_per_tick_);

        if (scale_ >= 1.0)
        {
            pub_->publish(target_);
            return;
        }

        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        out.translation() = current.translation() + scale_ * dp;
        out.linear() = (Eigen::Quaterniond(current.linear()).slerp(scale_, Eigen::Quaterniond(target.linear())))
                           .toRotationMatrix();
        pub_->publish(to_pose(out));
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Target limited to %.0f %% (%s - %s, clearance %.3f m, margin %.3f m)", 100.0 * scale_,
                             blocker.link_a.c_str(), blocker.link_b.c_str(), blocker.distance, margin_);
    }

    // Largest s in {1, 0.75, 0.5, 0.25, 0} for which moving by s * (dp, dw) keeps the required clearance
    double safe_scale(const moveit::core::RobotState& state, const Eigen::Isometry3d& root_in_model,
                      const Eigen::Vector3d& dp, const Eigen::Vector3d& dw, Clearance& blocker)
    {
        // Jacobian rows (linear, angular) are expressed in the model frame
        Eigen::MatrixXd jacobian;
        if (!state.getJacobian(jmg_, ee_model_, Eigen::Vector3d::Zero(), jacobian))
            return 0.0;
        Eigen::Matrix<double, 6, 1> gap;
        gap.head<3>() = root_in_model.linear() * dp;
        gap.tail<3>() = root_in_model.linear() * dw;

        const Eigen::MatrixXd regularised = jacobian * jacobian.transpose() +
                                            damping_ * damping_ * Eigen::MatrixXd::Identity(6, 6);
        Eigen::VectorXd dq = jacobian.transpose() * regularised.ldlt().solve(gap);
        const double largest = dq.cwiseAbs().maxCoeff();
        if (largest > max_joint_step_)
            dq *= max_joint_step_ / largest;

        std::vector<double> q;
        state.copyJointGroupPositions(jmg_, q);

        const Clearance now_clearance = clearance(state);
        for (const double s : {1.0, 0.75, 0.5, 0.25})
        {
            moveit::core::RobotState probe(state);
            std::vector<double> q_probe(q);
            for (std::size_t i = 0; i < q_probe.size() && i < static_cast<std::size_t>(dq.size()); ++i)
                q_probe[i] += s * dq[i];
            probe.setJointGroupPositions(jmg_, q_probe);
            probe.enforceBounds();
            probe.update();

            const Clearance candidate = clearance(probe);
            // enough room, or at least not closer than we already are (the arm may start inside the margin)
            if (candidate.distance >= margin_ || candidate.distance >= now_clearance.distance - 1e-3)
                return s;
            blocker = candidate;
        }
        if (blocker.link_a.empty())
            blocker = now_clearance;
        return 0.0;
    }

    rclcpp::Node::SharedPtr node_;
    std::string group_, ee_link_, root_link_;
    double margin_, lookahead_pos_, lookahead_rot_, damping_, max_joint_step_, recover_per_tick_, input_timeout_;

    robot_model_loader::RobotModelLoaderPtr loader_;
    planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
    moveit::core::RobotModelConstPtr model_;
    const moveit::core::JointModelGroup* jmg_ = nullptr;
    const moveit::core::LinkModel* ee_model_ = nullptr;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose target_;
    rclcpp::Time last_input_{0, 0, RCL_ROS_TIME};
    bool have_input_ = false;
    double scale_ = 1.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("cartesian_collision_guard", options);

    // The executor has to run while the guard waits for the joint states in its constructor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread ros_thread([&executor]() { executor.spin(); });

    CollisionGuard guard(node);
    ros_thread.join();
    rclcpp::shutdown();
    return 0;
}
