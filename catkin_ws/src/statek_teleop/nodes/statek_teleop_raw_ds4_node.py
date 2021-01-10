#!/usr/bin/env python
import rospy
import math
#import argparse
from ds4_driver.msg import Status
from statek_msgs.msg import Velocity

def ds4_callback(status, args):

    max_rpm_rads = args[0]

    left_vel = status.axis_left_y * max_rpm_rads
    right_vel = status.axis_right_y * max_rpm_rads

    cmd = Velocity()
    cmd.left = left_vel
    cmd.right = right_vel

    vel_cmd_publisher = args[1]
    vel_cmd_publisher.publish(cmd)

rospy.init_node("statek_teleop_raw_ds4", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
gamepad_name = rospy.get_param("~gamepad_name", "ds4")
max_rpm_rads = 2 * math.pi / 60 * rospy.get_param("~soft_max_rpm", 45)

vel_cmd_publisher = rospy.Publisher("/" + statek_name + "/motors/vel_cmd", Velocity, queue_size=10)

rospy.Subscriber("/" + gamepad_name + "/status", Status, ds4_callback, (max_rpm_rads, vel_cmd_publisher))

rospy.spin()
