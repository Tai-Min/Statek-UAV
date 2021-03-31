#!/usr/bin/env python
import rospy
import time

from ds4_driver.msg import Status
from geometry_msgs.msg import Twist

def ds4_callback(status, args):
    max_linear_velocity = args[0]
    max_angular_velocity = args[1]

    linear = status.axis_left_y * max_linear_velocity
    angular = status.axis_right_x * max_angular_velocity

    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular

    vel_cmd_publisher = args[2]
    vel_cmd_publisher.publish(cmd)

    time.sleep(0.1) # limit message rate to 10

rospy.init_node("teleop_raw_ds4", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
max_linear_velocity = rospy.get_param("~max_linear_velocity", 0)
max_angular_velocity = rospy.get_param("~max_angular_velocity", 0)

vel_cmd_publisher = rospy.Publisher("/" + statek_name + "/real_time/motors/twist_cmd", Twist, queue_size=1)
rospy.Subscriber("/" + statek_name + "/controller/status", Status, ds4_callback, (max_linear_velocity, max_angular_velocity, vel_cmd_publisher), queue_size=1)

rospy.spin()
