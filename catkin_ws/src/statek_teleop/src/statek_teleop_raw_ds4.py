#!/usr/bin/env python
import rospy
#import argparse
from ds4_driver.msg import Status
from std_msgs.msg import Float32

def ds4_callback(status, args):

    left_vel = status.axis_left_y
    right_vel = status.axis_right_y

    left_cmd = Float32()
    left_cmd.data = left_vel

    right_cmd = Float32()
    right_cmd.data = right_vel

    left_cmd_publisher = args[0]
    right_cmd_publisher = args[1]

    left_cmd_publisher.publish(left_cmd)
    right_cmd_publisher.publish(right_cmd)

rospy.init_node("statek_teleop_raw_ds4", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
gamepad_name = rospy.get_param("~gamepad_name", "ds4")

left_cmd_publisher = rospy.Publisher("/" + statek_name + "/vel_cmd_left", Float32, queue_size=10)
right_cmd_publisher = rospy.Publisher("/" + statek_name + "/vel_cmd_right", Float32, queue_size=10)

rospy.Subscriber("/" + gamepad_name + "/status", Status, ds4_callback, (left_cmd_publisher, right_cmd_publisher))

rospy.spin()
