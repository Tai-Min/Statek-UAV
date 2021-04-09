#!/usr/bin/env python
import rospy
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print 180 + yaw * 57.2957795

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('statek/real_time/imu', Imu, get_rotation)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()