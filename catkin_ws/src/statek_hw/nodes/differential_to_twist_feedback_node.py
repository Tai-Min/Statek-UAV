#!/usr/bin/env python
import rospy
from statek_msgs.msg import Encoder
from geometry_msgs.msg import TwistStamped

def differential_to_twist(left_ang_velocity, right_ang_velocity, params):
    left_lin_velocity = left_ang_velocity * params["wheel_radius"]
    right_lin_velocity = right_ang_velocity * params["wheel_radius"]

    linear = (left_lin_velocity + right_lin_velocity) / 2.0
    angular_scaled = linear - left_lin_velocity

    angular = angular_scaled / params["angular_scaling_factor"]

    msg = TwistStamped()

    msg.header.stamp = rospy.Time.now()
    msg.twist.linear.x = linear
    msg.twist.angular.z = angular

    return msg

def encoder_callback(msg, params):
    if params["side"] == "left":
        encoder_callback.left_ang_velocity = msg.velocity
    else:
        encoder_callback.right_ang_velocity = msg.velocity

    msg = differential_to_twist(encoder_callback.left_ang_velocity, encoder_callback.right_ang_velocity, params)

    params["twist_publisher"].publish(msg)

encoder_callback.left_ang_velocity = 0
encoder_callback.right_ang_velocity = 0

rospy.init_node("differential_to_twist_feedback", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")

wheel_max_angular_velocity = rospy.get_param("~wheel_max_angular_velocity", 0)
distance_between_wheels = rospy.get_param("~distance_between_wheels", 0)
angular_scaling_factor = distance_between_wheels / 2.0 # to transform from rad/s to m/s
wheel_radius = rospy.get_param("~wheel_radius", 0)

twist_publisher = rospy.Publisher("/" + statek_name + "/real_time/motors/twist_feedback", TwistStamped, queue_size=1)

params_dict_left = {
    "wheel_max_angular_velocity": wheel_max_angular_velocity,
    "angular_scaling_factor": angular_scaling_factor,
    "wheel_radius": wheel_radius,
    "twist_publisher": twist_publisher,
    "side": "left"
}

rospy.Subscriber("/" + statek_name + "/real_time/motors/left/encoder", Encoder, encoder_callback, params_dict_left, queue_size=1)

params_dict_right = {
    "wheel_max_angular_velocity": wheel_max_angular_velocity,
    "angular_scaling_factor": angular_scaling_factor,
    "wheel_radius": wheel_radius,
    "twist_publisher": twist_publisher,
    "side": "right"
}

rospy.Subscriber("/" + statek_name + "/real_time/motors/right/encoder", Encoder, encoder_callback, params_dict_right, queue_size=1)

rospy.spin()

