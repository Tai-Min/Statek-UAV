#!/usr/bin/env python
import rospy
from statek_hw.msg import Velocity
from geometry_msgs.msg import Twist
import time

def twist_to_differential(linear, angular, params):
    angular_scaled = angular * params["angular_scaling_factor"] # in m/s

    # to differential
    v_right_linear = linear + angular_scaled
    v_left_linear = linear - angular_scaled

    # to wheel angular speed
    v_right_angular = v_right_linear / params["wheel_radius"]
    v_left_angular = v_left_linear / params["wheel_radius"]

    # scale in case left/right angular velocity is bigger than max angular velocity
    if v_right_angular > params["wheel_max_angular_velocity"] and v_right_angular >= v_left_angular:
        scale_factor = params["wheel_max_angular_velocity"] / v_right_angular
        v_right_angular = params["wheel_max_angular_velocity"]
        v_left_angular *= scale_factor

    elif v_left_angular > params["wheel_max_angular_velocity"] and v_left_angular >= v_right_angular:
        scale_factor = params["wheel_max_angular_velocity"] / v_left_angular
        v_left_angular = params["wheel_max_angular_velocity"]
        v_right_angular *= scale_factor

    return {"left":v_left_angular, "right":v_right_angular}

def twist_callback(msg, params):
    linear = msg.linear.x
    angular = msg.angular.z

    velocities = twist_to_differential(linear, angular, params)

    cmd = Velocity()
    cmd.left = velocities["left"]
    cmd.right = velocities["right"]
    params["vel_cmd_publisher"].publish(cmd)

rospy.init_node("twist_to_differential", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")

wheel_max_angular_velocity = rospy.get_param("~wheel_max_angular_velocity", 0)
distance_between_wheels = rospy.get_param("~distance_between_wheels", 0)
angular_scaling_factor = distance_between_wheels / 2.0 # to transform from rad/s to m/s
wheel_radius = rospy.get_param("~wheel_radius", 0)

vel_cmd_publisher = rospy.Publisher("/" + statek_name + "/real_time/motors/vel_cmd", Velocity, queue_size=1)

params_dict = {
    "wheel_max_angular_velocity": wheel_max_angular_velocity,
    "angular_scaling_factor": angular_scaling_factor,
    "wheel_radius": wheel_radius,
    "vel_cmd_publisher": vel_cmd_publisher
}

rospy.Subscriber("/" + statek_name + "/real_time/motors/twist_cmd", Twist, twist_callback, params_dict, queue_size=1)

rospy.spin()