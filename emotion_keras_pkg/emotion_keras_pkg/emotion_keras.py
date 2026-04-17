import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt8MultiArray, String
import numpy as np
import multiprocessing as mp
import tf_keras
import tensorflow as tf
import time

timesteps=30
features=16
num_classes_1 = 6
num_classes_2 = 3

intens_list = ["LOW", "MED", "HI"]
emo_list = ["ANGER", "DISGUST", "FEAR", "HAPPY", "NEUTRAL", "SAD"]

def keras_process(weights_path, stop_flag, ready_flag, keras_input_q, keras_output_q):
    
    model = tf.keras.models.load_model(weights_path, compile=False)

    # First run to reduce subsequent latency
    model.predict(np.zeros((1,timesteps, features),dtype=np.float64))
    print("Model Done!")
    # Run until told to stop
    while stop_flag.value == 0:
        ready_flag.value = 1
        input_arr = np.array(keras_input_q.get())
        ready_flag.value = 0

        out = model.predict(np.reshape(input_arr, (1, timesteps, features)), verbose=0)

        keras_output_q.put(out)




class Emotion_Recognizer(Node):
    def __init__(self):
        super().__init__('emotion_recognizer')
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'emotion_array', 10)
        self.str_publisher_ = self.create_publisher(String, 'emotion_string', 10)
        
        self.stop_flag = mp.Value('i', 0) # used to tell keras node to stop
        self.ready_flag = mp.Value('i', 0) # used to signal that neural network is ready to recieve data
        self.keras_input_q = mp.Queue()
        self.keras_output_q = mp.Queue()



        weights_path = "/colcon_ws/tiago_ws/src/emotion_keras_pkg/networks/LSTM_Run21.keras"
        # weights_path = "/colcon_ws/tiago_ws/src/emotion_keras_pkg/networks/Conv_Run21.keras"
        print(weights_path)

        self.keras_process = mp.Process(target=keras_process, args=(
            weights_path,
            self.stop_flag, self.ready_flag,
            self.keras_input_q, self.keras_output_q
        ))
        
        self.keras_process.start()
        self.get_logger().info("Keras Process Started")

        # wait_ready_rate = self.create_rate(20.0, self.get_clock())
        # while self.ready_flag.value == 0:
        #     self.get_logger().info("Waiting!")
        #     print("Starting Sleep")
        #     wait_ready_rate.sleep()
        #     print("Done Sleep")

        # self.get_logger().info("Keras Ready")

        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            '/facial_input',
            self.listener_callback,
            10)
        
        self.queue = []

        self.timer = self.create_timer(1.0/50.0, self.check_keras, clock=self.get_clock()) # check for results at 50hz
        
    def __del__(self):
        self.stop_flag.value = 1
        self.keras_process.join()

    def listener_callback(self, msg):
        queue_length = len(self.queue)

        if queue_length >= timesteps:
            self.queue = self.queue[(queue_length-timesteps+1):]
        self.queue.append(msg.data)

        if len(self.queue) == timesteps and self.ready_flag.value and self.keras_input_q.qsize() == 0:
            self.keras_input_q.put(self.queue)
    
    def check_keras(self):
        if self.keras_output_q.qsize():

            data = self.keras_output_q.get()

            emo_out = data[0][0]
            intens_out = data[1][0]

            emo_cla = int(np.argmax(emo_out))
            intens_cla = int(np.argmax(intens_out))

            emo_pub_data = UInt8MultiArray()
            emo_pub_data.data = [emo_cla, int(emo_out[emo_cla]*100), intens_cla, int(intens_out[intens_cla]*100)]

            msg = String()
            msg.data = "%s: %0.2f\n%s: %0.2f" % (emo_list[emo_cla], float(emo_out[emo_cla]), intens_list[intens_cla], float(intens_out[intens_cla]))

            self.publisher_.publish(emo_pub_data)
            self.str_publisher_.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)
    emo_recogn = Emotion_Recognizer()
    emo_recogn.get_logger().info("Started Emo Recognizer")
    rclpy.spin(emo_recogn)
    emo_recogn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()