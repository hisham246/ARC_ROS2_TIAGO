import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt8MultiArray, String
import numpy as np
import multiprocessing as mp
import tf_keras
import tensorflow as tf
import emotion_keras_pkg.Custom_Accuracy as Custom_Accuracy
import emotion_keras_pkg.Custom_Loss as Custom_Loss
import time

timesteps=30
features=16
num_classes_1 = 6
num_classes_2 = 3

intens_list = ["LOW", "MED", "HI"]
emo_list = ["ANGER", "DISGUST", "FEAR", "HAPPY", "NEUTRAL", "SAD"]

def keras_process(weights_path, model_type, load_model, stop_flag, ready_flag, keras_input_q, keras_output_q):
    
    model = None
    if load_model:
        model = tf_keras.models.load_model(weights_path, compile=False,
                                        custom_objects={'CustomMaskedCategoricalCrossentropy': Custom_Loss.CustomMaskedCategoricalCrossentropy,
                                                        'CustomAccuracy': Custom_Accuracy.CustomAccuracy})
    else:
        # Define the input layer
        input_sequence = tf_keras.layers.Input(shape=(timesteps, features)) # timesteps: length of sequence, features: number of features per timestep

        output_classification_1 = None
        output_classification_2 = None

        
        if model_type==0:
            input_reshape = tf.expand_dims(input_sequence, axis=-1)
            L1 = tf_keras.layers.Conv2D(16, (7, 7), activation='relu', padding='same')(input_reshape)
            Layer = tf_keras.layers.Conv2D(16, (7, 7), activation='relu', padding='same')(L1)
            Layer = tf_keras.layers.Conv2D(16, (7, 7), activation='relu', padding='same')(Layer)
            Layer = tf_keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same')(Layer)
            Layer = tf_keras.layers.Conv2D(16, (3, 3), activation='relu')(Layer)
            Layer = tf_keras.layers.Conv2D(16, (3, 3), activation='relu')(Layer)
            Layer = tf_keras.layers.Flatten()(Layer)

            conv_output = tf_keras.layers.Dense(units=128, activation='relu')(Layer)

            # Define the first classification head
            output_classification_1 = tf_keras.layers.Dense(units=num_classes_1, activation='softmax', name='classification_1')(conv_output)

            # Define the second classification head
            output_classification_2 = tf_keras.layers.Dense(units=num_classes_2, activation='softmax', name='classification_2')(conv_output)

        elif model_type==1:
            # Define the shared LSTM layer
            lstm_output = tf_keras.layers.LSTM(units=128)(input_sequence) # units: dimensionality of the output space

            lstm_output = tf_keras.layers.Dense(units=128, activation='relu')(lstm_output)

            # Define the first classification head
            output_classification_1 = tf_keras.layers.Dense(units=num_classes_1, activation='softmax', name='classification_1')(lstm_output)

            # Define the second classification head
            output_classification_2 = tf_keras.layers.Dense(units=num_classes_2, activation='softmax', name='classification_2')(lstm_output)

        elif model_type==2:
            # num_pyr = 4
            filter_sizes = [1, 1, 1, 1, 1, 1, 0.95, 0.9, 0.8, 0.75, 0.6, 0.5, 0.4, 0.33333, 0.25, 0.1]
            num_filters = 32

            # Define the input layer
            input_sequence = tf_keras.Input(shape=(timesteps, features)) # timesteps: length of sequence, features: number of features per timestep

            input_reshape = tf.keras.ops.expand_dims(input_sequence, axis=-1)

            Layer = tf_keras.Conv2D(num_filters, (1, features), activation='relu')(input_reshape)

            for scale in filter_sizes:
                shape = (int(scale*timesteps), int(features))
                Layer = tf_keras.Concatenate(axis=1)([Layer, tf_keras.Conv2D(num_filters, shape, activation='relu')(input_reshape)])

            Layer = tf_keras.Flatten()(Layer)

            Layer = tf_keras.Dense(units=512, activation='relu')(Layer)
            Layer = tf_keras.Dense(units=256, activation='relu')(Layer)
            Layer = tf_keras.Dense(units=128, activation='relu')(Layer)
            Layer = tf_keras.Dense(units=128, activation='relu')(Layer)
            conv_output = tf_keras.Dense(units=64, activation='relu')(Layer)

            # Define the first classification head
            output_classification_1 = tf_keras.Dense(units=num_classes_1, activation='softmax', name='classification_1')(conv_output)

            # Define the second classification head
            output_classification_2 = tf_keras.Dense(units=num_classes_2, activation='softmax', name='classification_2')(conv_output)

        # Create model
        model = tf_keras.Model(inputs=[input_sequence], outputs=[output_classification_1, output_classification_2])

        # Load weights
        try:
            model.load_weights(weights_path)
            print("Weights loaded successfully.")
        except tf.errors.NotFoundError:
            print("Weights file not found. Ensure 'my_model_weights.h5' exists.")
        
        # Compile model
        model.compile()

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



        weights_path = "/home/adam/colcon_ws/colcon_ws/src/emotion_keras_pkg/networks/LSTM_Run21.keras"
        # weights_path = "/home/adam/colcon_ws/colcon_ws/src/emotion_keras_pkg/networks/Conv_Run21.keras"
        print(weights_path)
        model_type = 0 #0 for conv window, 1 for lstm
        load_model = True

        self.keras_process = mp.Process(target=keras_process, args=(
            weights_path, model_type, load_model,
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