#!/usr/bin/env python
import rospy

from statek_msgs.msg import Encoder

def encoder_callback(status, args):
    pass


rospy.init_node("dynamic_transform_publisher", anonymous=True)

left_wheels_transform_publisher = rospy.Publisher("/" + statek_name + "/vel_cmd_left", Float32, queue_size=10)
right_wheels_transform_publisher = rospy.Publisher("/" + statek_name + "/vel_cmd_right", Float32, queue_size=10)

statek_name = rospy.get_param("~statek_name", "statek")

rospy.Subscriber("/" + statek_name + "/encoder_filtered_left", Encoder, encoder_callback, ("left"))
rospy.Subscriber("/" + statek_name + "/encoder_filtered_right", Encoder, encoder_callback, ("right"))

rospy.spin()
