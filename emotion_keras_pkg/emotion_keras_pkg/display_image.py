import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.declare_parameter('image_topic', 'face_image')
        param_obj = self.get_parameter('image_topic') 
        self.subscription = self.create_subscription(
            Image, param_obj.value, self.listener_callback, 10)
        self.bridge = CvBridge()
        cv2.namedWindow("face", cv2.WINDOW_NORMAL)

        cv2.setWindowProperty("face", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        cv2.imshow("face", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()