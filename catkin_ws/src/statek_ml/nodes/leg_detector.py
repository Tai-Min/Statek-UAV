#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
#import tensorflow as tf
import sys
import cv2
import numpy as np

def load_model():
    model_path = rospkg.RosPack().get_path("statek_ml")
    model_path += "/models/lidar_net"
    sys.path.insert(1, model_path)

    net = __import__("net")
    net = net.PeTraNet()

    preprocessor = __import__("dataset_preprocessing")
    preprocessor = preprocessor.preprocess_real_data

    #ckpt = tf.train.latest_checkpoint(model_path + "/.tf_ckpts")
    #ckpt = tf.train.Checkpoint(net=net).restore(ckpt).expect_partial()

    return net, preprocessor

def scan_callback(msg):
    frame = np.array(list(msg.data))
    frame = np.reshape(frame, (1, msg.height, msg.width))
    print(frame.shape)

    #frame = scan_callback.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #print(frame)

# Init ROS.
rospy.init_node('lidar_fconv_dataset_collector')
statek_name = rospy.get_param("~statek_name", "statek")

# Init lidar image subscriber.
rospy.Subscriber("/" + statek_name + "/laser/scan_img", Image, scan_callback)

# Stuff for laser image callback.
#scan_callback.bridge = CvBridge()
scan_callback.leg_publisher = rospy.Publisher("/" + statek_name + "/laser/legs_img", Image, queue_size=1)
#scan_callback.net = load_model()

rospy.spin()