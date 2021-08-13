#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import tensorflow as tf
import sys
import cv2
import numpy as np

def load_model():
    model_path = rospkg.RosPack().get_path("statek_ml")
    model_path += "/models/lidar_net"
    sys.path.insert(1, model_path)

    net = __import__("net")
    net = net.PeTraNet()

    preprocessor = __import__("dataset_processing")
    preprocessor = preprocessor.preprocess_input_sample

    ckpt = tf.train.latest_checkpoint(model_path + "/.tf_ckpts")
    ckpt = tf.train.Checkpoint(net=net).restore(ckpt).expect_partial()

    return net, preprocessor

# Can't get cv_bridge to work on python3 and Jeston Nano D:
def msg_to_cv(msg):
    frame = np.array(list(msg.data))
    frame = np.reshape(frame, (1, msg.height, msg.width))
    return frame

def predition_to_msg():
    pass

def scan_callback(msg):
    frame = msg_to_cv(msg)
    frame = scan_callback.preprocessor(frame)
    prediction = scan_callback.net(frame)

#tf.config.gpu.set_per_process_memory_fraction(0.5)
#tf.config.gpu.set_per_process_memory_growth(True)

# Init ROS.
rospy.init_node('lidar_fconv_dataset_collector')
statek_name = rospy.get_param("~statek_name", "statek")

# Stuff for laser image callback.
scan_callback.leg_publisher = rospy.Publisher("/" + statek_name + "/laser/legs_img", Image, queue_size=1)
scan_callback.net, scan_callback.preprocessor = load_model()

# Init lidar image subscriber.
rospy.Subscriber("/" + statek_name + "/laser/scan_img", Image, scan_callback)

rospy.spin()