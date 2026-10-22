#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <custom_teleop_messages/msg/xr_mod.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class bridge : public rclcpp::Node
{
    public:
        bridge()
        : Node("bridge")
        {
            std::string L_pose_top, R_pose_top, Head_pose_top, L_grip_top, R_grip_top, XR_top;

            this->declare_parameter("base_link", std::string("base_link"));
            base_link_ = this->get_parameter("base_link").as_string();

            this->declare_parameter("left_pose_topic", std::string("left_arm_pose"));
            L_pose_top = this->get_parameter("left_pose_topic").as_string();

            this->declare_parameter("right_pose_topic", std::string("right_arm_pose"));
            R_pose_top = this->get_parameter("right_pose_topic").as_string();

            this->declare_parameter("head_pose_topic", std::string("head_pose"));
            Head_pose_top = this->get_parameter("head_pose_topic").as_string();

            this->declare_parameter("left_grip_topic", std::string("left_arm_grip"));
            L_grip_top = this->get_parameter("left_grip_topic").as_string();

            this->declare_parameter("right_grip_topic", std::string("right_arm_grip"));
            R_grip_top = this->get_parameter("right_grip_topic").as_string();

            this->declare_parameter("sub_topic", std::string("XR_Input"));
            XR_top = this->get_parameter("sub_topic").as_string();

            this->declare_parameter("enable_left", true);
            this->declare_parameter("enable_right", true);
            this->declare_parameter("enable_head", true);
            enable_left_ = this->get_parameter("enable_left").as_bool();
            enable_right_ = this->get_parameter("enable_right").as_bool();
            enable_head_ = this->get_parameter("enable_head").as_bool();

            // Relative mode: the arm follows the change of the controller pose from the moment tracking
            // (re)started, starting from the arm's current tool pose, instead of the absolute controller pose
            this->declare_parameter("relative_mode", false);
            relative_mode_ = this->get_parameter("relative_mode").as_bool();
            this->declare_parameter("relative_timeout", 0.3);
            relative_timeout_ = this->get_parameter("relative_timeout").as_double();
            this->declare_parameter("left_tool_frame", std::string("arm_left_tool_link"));
            left_.tool_frame = this->get_parameter("left_tool_frame").as_string();
            this->declare_parameter("right_tool_frame", std::string("arm_right_tool_link"));
            right_.tool_frame = this->get_parameter("right_tool_frame").as_string();
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            tool_left_trans_ = get_transformation("left_tool_trans");
            tool_right_trans_ = get_transformation("right_tool_trans");
            tool_head_trans_ = get_transformation("head_tool_trans");

            world_left_trans_ = get_transformation("left_world_trans");
            world_right_trans_ = get_transformation("right_world_trans");
            world_head_trans_ = get_transformation("head_world_trans");


            L_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(L_pose_top, 1);
            R_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(R_pose_top, 1);
            Head_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(Head_pose_top, 1);
            L_grip_pub_ = this->create_publisher<std_msgs::msg::Float32>(L_grip_top, 1);
            R_grip_pub_ = this->create_publisher<std_msgs::msg::Float32>(R_grip_top, 1);

            XR_sub_ = this->create_subscription<custom_teleop_messages::msg::XRMod>(XR_top, 1,
                std::bind(&bridge::callback, this,std::placeholders::_1));
        }
    private:
        void callback(const custom_teleop_messages::msg::XRMod::ConstPtr message)
        {
            rclcpp::Time time = rclcpp::Clock().now();
            
            std_msgs::msg::Float32 left_squeeze, right_squeeze;
            geometry_msgs::msg::PoseStamped Left, Right, Head;

            Left.header.frame_id = base_link_;
            Right.header.frame_id = base_link_;
            Head.header.frame_id = base_link_;

            // Zero stamp = latest transform, so the planner's TF lookup works with sim time as well as wall time
            Left.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
            Right.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
            Head.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
            
            // A device that is not tracked reports a pose at the origin; only the enabled devices have to be tracked.
            if ((enable_left_ && is_empty(message->left_controller_pose)) ||
                (enable_right_ && is_empty(message->right_controller_pose)) ||
                (enable_head_ && is_empty(message->hmd_pose)))
            {
                return;
            }

            if (enable_left_)
            {
                transform_pose(message->left_controller_pose, Left.pose, world_left_trans_, tool_left_trans_);
                if (relative_mode_ && !make_relative(left_, Left.pose, time))
                    return;
                left_squeeze.data = message->left_squeeze;
                L_pose_pub_->publish(Left);
                L_grip_pub_->publish(left_squeeze);
            }
            if (enable_right_)
            {
                transform_pose(message->right_controller_pose, Right.pose, world_right_trans_, tool_right_trans_);
                if (relative_mode_ && !make_relative(right_, Right.pose, time))
                    return;
                right_squeeze.data = message->right_squeeze;
                R_pose_pub_->publish(Right);
                R_grip_pub_->publish(right_squeeze);
            }
            if (enable_head_)
            {
                transform_pose(message->hmd_pose, Head.pose, world_head_trans_, tool_head_trans_);
                Head_pose_pub_->publish(Head);
            }
        }

    struct Clutch
    {
        std::string tool_frame;
        bool engaged = false;
        rclcpp::Time last_valid{0, 0, RCL_SYSTEM_TIME};
        Eigen::Affine3d controller_start, tool_start;
    };

    // Replaces the controller pose with the arm's start pose moved by the controller's motion since the latch
    bool make_relative(Clutch &clutch, geometry_msgs::msg::Pose &pose, const rclcpp::Time &now)
    {
        Eigen::Affine3d controller;
        tf2::convert(pose, controller);

        if (!clutch.engaged || (now - clutch.last_valid).seconds() > relative_timeout_)
        {
            try
            {
                clutch.tool_start = tf2::transformToEigen(
                    tf_buffer_->lookupTransform(base_link_, clutch.tool_frame, tf2::TimePointZero));
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Cannot latch " << clutch.tool_frame << ": " << ex.what());
                return false;
            }
            clutch.controller_start = controller;
            clutch.engaged = true;
        }
        clutch.last_valid = now;

        Eigen::Affine3d target = Eigen::Affine3d::Identity();
        target.linear() = (controller.linear() * clutch.controller_start.linear().inverse()) * clutch.tool_start.linear();
        target.translation() = clutch.tool_start.translation() + (controller.translation() - clutch.controller_start.translation());
        tf2::convert(target, pose);
        return true;
    }

    static bool is_empty(const geometry_msgs::msg::Pose &pose)
    {
        return (fabs(pose.position.x) + fabs(pose.position.y) + fabs(pose.position.z)) < 0.001;
    }

    void transform_pose(geometry_msgs::msg::Pose input, geometry_msgs::msg::Pose &output, Eigen::Affine3d world_transform, Eigen::Affine3d tool_transform)
    {
        Eigen::Affine3d temp;

        tf2::convert(input, temp);

        temp = world_transform * temp * tool_transform;

        tf2::convert(temp, output);
    }

    Eigen::Affine3d get_transformation(std::string param)
    {
        geometry_msgs::msg::Pose temp;
        double roll, pitch, yaw;
        this->declare_parameter(param+".x", 0.0);
        temp.position.x = this->get_parameter(param+".x").as_double();
        this->declare_parameter(param+".y", 0.0);
        temp.position.y = this->get_parameter(param+".y").as_double();
        this->declare_parameter(param+".z", 0.0);
        temp.position.z = this->get_parameter(param+".z").as_double();

        this->declare_parameter(param+".roll", 0.0);
        roll = this->get_parameter(param+".roll").as_double();
        this->declare_parameter(param+".pitch", 0.0);
        pitch = this->get_parameter(param+".pitch").as_double();
        this->declare_parameter(param+".yaw", 0.0);
        yaw = this->get_parameter(param+".yaw").as_double();

        temp.orientation = quaternion_from_rpy(roll, pitch, yaw);
        
        Eigen::Affine3d output;

        tf2::convert(temp, output);

        return output;
    }

    geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    rclcpp::Subscription<custom_teleop_messages::msg::XRMod>::SharedPtr XR_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr L_pose_pub_, R_pose_pub_, Head_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr L_grip_pub_, R_grip_pub_;
    
    std::string base_link_;
    bool enable_left_, enable_right_, enable_head_;
    bool relative_mode_;
    double relative_timeout_;
    Clutch left_, right_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    Eigen::Affine3d tool_left_trans_, tool_right_trans_, tool_head_trans_, world_left_trans_, world_right_trans_, world_head_trans_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bridge>());
  rclcpp::shutdown();
  return 0;
}