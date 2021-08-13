#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf
import sys

def load_model():
    model_path = rospkg.RosPack().get_path("statek_ml")
    model_path += "/models/lidar_net"
    sys.path.insert(1, model_path)

    net = __import__("net")
    net = net.PeTraNet()

    ckpt = tf.train.latest_checkpoint(model_path + "/")
    ckpt = tf.train.Checkpoint(net=net).restore(ckpt).expect_partial()

    return net

def scan_callback(msg):
    
scan_callback.net = load_model()


bridge = CvBridge()

# Init ROS.
rospy.init_node('lidar_fconv_dataset_collector')
statek_name = rospy.get_param("~statek_name", "statek")

# Init lidar image subscriber.
rospy.Subscriber("/" + statek_name + "/laser/scan_img", Image, scan_callback)