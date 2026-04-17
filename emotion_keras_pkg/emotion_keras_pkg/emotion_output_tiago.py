import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import numpy as np
from hri_msgs.msg import Expression


class CycleExpressions(Node):
    def __init__(self):
        super().__init__('cycle_expressions')
        self.publisher_ = self.create_publisher(Expression,
                                                '/robot_face/expression', 1)
        self.subscriber_ = self.create_subscription(UInt8MultiArray, 'emotion_array', self.publish_expression, 10)
        self.expressions = ['angry', 'disgusted', 'scared',
                            'happy', 'neutral', 'sad']
        
        self.buffer_class = []
        self.buffer_perc = []
        self.buffer_size = 5

        self.publish_rate = 1.0
        self.start_time = self.get_clock().now()

    def publish_expression(self, msg):

        self.buffer_class.append(msg.data[0])
        self.buffer_perc.append(msg.data[1])

        if len(self.buffer_class) > self.buffer_size:
            self.buffer_class = self.buffer_class[1:]
            self.buffer_perc = self.buffer_perc[1:]

        cur_time = self.get_clock().now()

        if (self.start_time-cur_time).nanoseconds > 1.0e9/self.publish_rate:
            outmsg = Expression()
            
            outmsg.expression = self.expressions[self.buffer_class[np.argmax(self.buffer_perc)]]

            self.start_time = cur_time
            
            self.publisher_.publish(outmsg)


if __name__ == '__main__':
    rclpy.init()
    node = CycleExpressions()
    rclpy.spin(node)
    rclpy.shutdown()