#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

class gripper_traj_interface : public rclcpp::Node
{
    public:
        gripper_traj_interface(): Node("gripper_traj_node")

        {
            std::string sub_topic, pub_topic, pub_type;

            this->declare_parameter("sub_top", "sub");
            this->declare_parameter("pub_top", "pub");
            this->declare_parameter("pub_type", "joint_trajectory");
            this->declare_parameter("finger_joints", std::vector<std::string>());

            sub_topic = this->get_parameter("sub_top").as_string();
            pub_topic = this->get_parameter("pub_top").as_string();
            pub_type = this->get_parameter("pub_type").as_string();
            joints_ = this->get_parameter("finger_joints").as_string_array();

            for (auto& joint : joints_)
            {
                // ros::NodeHandle priv_nh(ph, joint);
                // ROS_INFO_STREAM(priv_nh.getNamespace());
                std::string joint_name;
                double open_angle, closed_angle, max_speed;

                this->declare_parameter(joint+".joint_name", "joint");
                joint_name = this->get_parameter(joint+".joint_name").as_string();

                this->declare_parameter(joint+".open_angle", 0.0);
                open_angle = this->get_parameter(joint+".open_angle").as_double();

                this->declare_parameter(joint+".closed_angle", 1.0);
                closed_angle = this->get_parameter(joint+".closed_angle").as_double();
                
                this->declare_parameter(joint+".max_speed", 1.0);
                max_speed = this->get_parameter(joint+".max_speed").as_double();

                joint_names_.push_back(joint_name);
                open_angles_.push_back(open_angle);
                closed_angles_.push_back(closed_angle);
                max_speeds_.push_back(max_speed);
                
            }
            RCLCPP_WARN_STREAM(this->get_logger(), pub_type);

            if (!pub_type.compare("joint_trajectory"))
            {
                pub_traj_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_topic, 10);
                pub_type_enum_ = 1;
            }
            else if (!pub_type.compare("gripper_command"))
            {
                pub_grip_ = this->create_publisher<control_msgs::msg::GripperCommand>(pub_topic, 10);
                pub_type_enum_ = 2;
            }
            else
            {
                exit(-1);
            }
            RCLCPP_WARN_STREAM(this->get_logger(), pub_type_enum_);

            sub_ = this->create_subscription<std_msgs::msg::Float32>(
                sub_topic, 10, std::bind(&gripper_traj_interface::callback, this, _1));
        }
    private:
        void callback(const std_msgs::msg::Float32::SharedPtr message)
        {
            rclcpp::Time cur_time = this->get_clock()->now();
            
            if (pub_type_enum_ == 1)
            {
                trajectory_msgs::msg::JointTrajectory new_traj;
                // new_traj.joint_names = joints_;
                new_traj.points.push_back(trajectory_msgs::msg::JointTrajectoryPoint());
                new_traj.points[0].time_from_start = rclcpp::Duration(0, 0);
                new_traj.header.frame_id = "base_link";
                for (int i = 0; i < joint_names_.size(); i++)
                {
                    // ROS_INFO_STREAM(finger.closed_angle);
                    new_traj.joint_names.push_back(joint_names_[i]);
                    new_traj.points[0].positions.push_back(open_angles_[i]+((closed_angles_[i]-open_angles_[i])*message->data));
                }

                pub_traj_->publish(new_traj);
            }
            else if (pub_type_enum_ == 2)
            {
                control_msgs::msg::GripperCommand grip_cmd;
                grip_cmd.position = open_angles_[0]+((closed_angles_[0]-open_angles_[0])*message->data);
                grip_cmd.max_effort = -1;

                pub_grip_->publish(grip_cmd);
            }
        }


    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj_;
    rclcpp::Publisher<control_msgs::msg::GripperCommand>::SharedPtr pub_grip_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;

    uint16_t pub_type_enum_ = 0;
    // std::vector<finger_struct> finger_data;
    std::vector<std::string> joint_names_;
    std::vector<double> open_angles_, closed_angles_, max_speeds_;
    std::vector<std::string> joints_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gripper_traj_interface>());
    rclcpp::shutdown();
    return 0;
}