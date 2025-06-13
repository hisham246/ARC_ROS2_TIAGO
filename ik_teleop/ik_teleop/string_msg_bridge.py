#!/usr/bin/env python3

import importlib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from opentera_webrtc_ros_msgs.msg import PeerData

from rclpy_message_converter import json_message_converter

class message_bridge(Node):
    def __init__(self):
        super().__init__('message_bridge')

        self.ros_topic_types = {}
        self.pubs = {}

        string_sub_topic = self.declare_parameter("sub_topic", "data_incomming").get_parameter_value().string_value

        self.main_sub = self.create_subscription(PeerData, string_sub_topic, self.sub_callback, 10)

    def get_message_type(self, msg_type_str):
        
        # msg_type_str = peer_data
        module_name, class_name = msg_type_str.rsplit('/', 1)
        module = importlib.import_module(f"{module_name}.msg")
        return getattr(module, class_name)
    
    def sub_callback(self, msg):
        
        # try:
            topic, top_type, data = msg.data.split('!', 2)

            if topic not in self.ros_topic_types:
                self.ros_topic_types[topic] = self.get_message_type(top_type)
            if topic not in self.pubs:
                self.pubs[topic] = self.create_publisher(self.ros_topic_types[topic], topic, 10)
                
            self.pubs[topic].publish(json_message_converter.convert_json_to_ros_message(self.ros_topic_types[topic], data))

        # except:
        #     self.get_logger().error("Unable to process message " + msg.data)
            # rospy.logerr("Unable to process message" + msg.data)
        

def main(args=None):
    rclpy.init()
    bridge = message_bridge()

    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()