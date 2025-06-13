#include "rclcpp/rclcpp.hpp"

#include "opentera_webrtc_ros_msgs/msg/peer_audio.hpp"
#include "audio_utils_msgs/msg/audio_frame.hpp"
using std::placeholders::_1;

class relay_node : public rclcpp::Node
{
public:
    relay_node(): Node("opentera_audio_relay")
    {
        this->declare_parameter("sub_top", "sub");
        this->declare_parameter("pub_top", "pub");

        sub_top_ = this->get_parameter("sub_top").as_string();
        pub_top_ = this->get_parameter("pub_top").as_string();
 
        sub_ = this->create_subscription<opentera_webrtc_ros_msgs::msg::PeerAudio>(
            sub_top_, 10, std::bind(&relay_node::chatterCallback, this, _1));

        pub_ = this->create_publisher<audio_utils_msgs::msg::AudioFrame>(pub_top_, 10);
    }

protected:
    void chatterCallback(const opentera_webrtc_ros_msgs::msg::PeerAudio::SharedPtr msg)
    {
        pub_->publish(msg->frame);
    }
    
    std::string sub_top_, pub_top_;
    rclcpp::Publisher<audio_utils_msgs::msg::AudioFrame>::SharedPtr pub_;
    rclcpp::Subscription<opentera_webrtc_ros_msgs::msg::PeerAudio>::SharedPtr sub_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<relay_node>());
    rclcpp::shutdown();
    return 0;
}