#!/usr/bin/env python

import time
import rospy
from statek_msgs.srv import SetMotorParams
from statek_msgs.srv import SetMotorParamsRequest

def send_motor_params(namespace, param_service, params):
    full_service_name = "/" + namespace + param_service
    rospy.loginfo("Sending parameters to %s" % (full_service_name))
    rospy.wait_for_service(full_service_name)
    try:
        service = rospy.ServiceProxy(full_service_name, SetMotorParams)

        req = SetMotorParamsRequest()
        req.loop_update_rate_ms = params["loop_update_rate_ms"]
        req.wheel_max_angular_velocity = params["wheel_max_angular_velocity"]
        req.kp = params["kp"]
        req.ki = params["ki"]
        req.kd = params["kd"]

        res = service(req)
        if res.ok == False:
            rospy.logwarn("Failed to send parameters.")
    except:
        pass # STM still receives parameters D: so not sure why this fires.
        #rospy.logwarn("Exception occured! Failed to send parameters.")

rospy.init_node("real_time_param_sender", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
left_motor_param_service_name = rospy.get_param("~left_motor_param_service_name", "/real_time/motors/left/set_params")
right_motor_param_service_name = rospy.get_param("~right_motor_param_service_name", "/real_time/motors/right/set_params")

left_motor_pid = rospy.get_param("~left_motor_pid", [0,0,0])
right_motor_pid = rospy.get_param("~right_motor_pid", [0,0,0])

left_motor_params = {
    "loop_update_rate_ms": rospy.get_param("~loop_update_rate_ms", 0),
    "wheel_max_angular_velocity": rospy.get_param("~wheel_max_angular_velocity", 0),
    "kp": left_motor_pid[0],
    "ki": left_motor_pid[1],
    "kd": left_motor_pid[2]
}

right_motor_params = {
    "loop_update_rate_ms": rospy.get_param("~loop_update_rate_ms", 0),
    "wheel_max_angular_velocity": rospy.get_param("~wheel_max_angular_velocity", 0),
    "kp": right_motor_pid[0],
    "ki": right_motor_pid[1],
    "kd": right_motor_pid[2]
}

time.sleep(10)
send_motor_params(statek_name, left_motor_param_service_name, left_motor_params)
time.sleep(1)
send_motor_params(statek_name, right_motor_param_service_name, right_motor_params)
time.sleep(1)
# TODO: imu params...
time.sleep(1)
# TODO: odom params...
