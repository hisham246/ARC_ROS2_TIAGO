#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "opentera_webrtc_ros_msgs/msg/peer_image.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class relay_node : public rclcpp::Node
{
public:
    relay_node(): Node("opentera_video_relay")
    {
        this->declare_parameter("sub_top", "sub");
        this->declare_parameter("pub_top", "pub");

        sub_top_ = this->get_parameter("sub_top").as_string();
        pub_top_ = this->get_parameter("pub_top").as_string();

        sub_ = this->create_subscription<opentera_webrtc_ros_msgs::msg::PeerImage>(
            sub_top_, 10, std::bind(&relay_node::chatterCallback, this, _1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(pub_top_, 10);
    }

protected:
    void chatterCallback(const opentera_webrtc_ros_msgs::msg::PeerImage::SharedPtr msg)
    {
        pub_->publish(msg->frame);
    }
    
    std::string sub_top_, pub_top_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<opentera_webrtc_ros_msgs::msg::PeerImage>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<relay_node>());
    rclcpp::shutdown();
    return 0;
}