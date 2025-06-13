#!/usr/bin/env python3

import importlib
import rclpy
import rclpy.node
from std_msgs.msg import String

from rclpy_message_converter import json_message_converter

class message_bridge(rclpy.node.Node):
    def __init__(self):
        super().__init__('message_bridge')
        self.subs = []
        
        subs = self.declare_parameter('subs', []).get_parameter_value().string_array_value

        pub_topic = self.declare_parameter("pub_topic", "data_outgoing").get_parameter_value().string_value

        self.main_pub = self.create_publisher(String, pub_topic, 10)

        for sub in subs:
            sub_topic, msg_type, sub_type = self.get_parameters(sub)
            self.subs.append(self.create_subscription(sub_type,sub_topic, lambda msg: self.sub_callback(msg, sub_topic, msg_type), 10))
    
    def get_parameters(self, namespace):
        
        topic = self.declare_parameter(namespace+"/topic").get_parameter_value().string_value

        msg_type_string = self.declare_parameter(namespace+"/msg_type").get_parameter_value().string_array_value

        return topic, msg_type_string, self.get_message_type(msg_type_string)

    def get_message_type(self, msg_type_str):
        module_name, class_name = msg_type_str.rsplit('/', 1)
        module = importlib.import_module(f"{module_name}.msg")
        return getattr(module, class_name)
    
    def sub_callback(self, msg, sub_topic, msg_type):
        new_msg = String()
        new_msg.data = sub_topic+"!"+msg_type+"!"+json_message_converter.convert_ros_message_to_json(msg)
        self.main_pub.publish(new_msg)

def main(args=None):
    rclpy.init()
    bridge = message_bridge()
    rclpy.spin(bridge)


if __name__ == '__main__':
    main()