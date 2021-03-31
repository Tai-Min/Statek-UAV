#!/usr/bin/env python
import rospy
import math
import time

#import argparse
from ds4_driver.msg import Status
from statek_msgs.msg import Velocity

def ds4_callback(status, args):
    wheel_max_angular_velocity = args[0]

    left_vel = status.axis_left_y * wheel_max_angular_velocity
    right_vel = status.axis_right_y * wheel_max_angular_velocity

    cmd = Velocity()
    cmd.left = left_vel
    cmd.right = right_vel

    vel_cmd_publisher = args[1]
    vel_cmd_publisher.publish(cmd)

    time.sleep(0.1) # limit message rate to 10

rospy.init_node("teleop_raw_ds4", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
gamepad_name = rospy.get_param("~gamepad_name", "ds4")
wheel_max_angular_velocity = rospy.get_param("~wheel_max_angular_velocity", 0)

vel_cmd_publisher = rospy.Publisher("/" + statek_name + "/motors/vel_cmd", Velocity, queue_size=1)
rospy.Subscriber("/" + gamepad_name + "/status", Status, ds4_callback, (wheel_max_angular_velocity, vel_cmd_publisher), queue_size=1)

rospy.spin()
