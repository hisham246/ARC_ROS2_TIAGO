#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <custom_teleop_messages/msg/xr_mod.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

            Left.header.stamp = time;
            Right.header.stamp = time;
            Head.header.stamp = time;
            
            if (((fabs(message->left_controller_pose.position.x) + fabs(message->left_controller_pose.position.y)+fabs(message->left_controller_pose.position.z)) < 0.001) || 
                ((fabs(message->right_controller_pose.position.x) + fabs(message->right_controller_pose.position.y)+fabs(message->right_controller_pose.position.z)) < 0.001) || 
                ((fabs(message->hmd_pose.position.x) + fabs(message->hmd_pose.position.y)+fabs(message->hmd_pose.position.z)) < 0.001))
                {
                    // RCLCPP_WARN_STREAM(this->get_logger(), "Empty Message!");
                    return;
                }

            transform_pose(message->left_controller_pose, Left.pose, world_left_trans_, tool_left_trans_);
            transform_pose(message->right_controller_pose, Right.pose, world_right_trans_, tool_right_trans_);
            transform_pose(message->hmd_pose, Head.pose, world_head_trans_, tool_head_trans_);
            
            // Left.pose = message->left_controller_pose;
            // Right.pose = message->right_controller_pose;
            // Head.pose = message->hmd_pose;

            left_squeeze.data = message->left_squeeze;
            right_squeeze.data = message->right_squeeze;

            L_pose_pub_->publish(Left);
            R_pose_pub_->publish(Right);
            Head_pose_pub_->publish(Head);

            L_grip_pub_->publish(left_squeeze);
            R_grip_pub_->publish(right_squeeze);

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

    Eigen::Affine3d tool_left_trans_, tool_right_trans_, tool_head_trans_, world_left_trans_, world_right_trans_, world_head_trans_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bridge>());
  rclcpp::shutdown();
  return 0;
}