#!/usr/bin/env python
import time
import rospy
from statek_hw.srv import SetMotorParams
from statek_hw.srv import SetImuParams
from statek_hw.srv import SetOdomParams

from statek_hw.srv import SetMotorParamsRequest
from statek_hw.srv import SetImuParamsRequest
from statek_hw.srv import SetOdomParamsRequest

def send_motor_params(namespace, param_service, params):
    full_service_name = "/" + namespace + param_service
    rospy.loginfo("Sending parameters to %s" % (full_service_name))
    rospy.wait_for_service(full_service_name)
    try:
        service = rospy.ServiceProxy(full_service_name, SetMotorParams)

        req = SetMotorParamsRequest()
        req.loop_update_rate_ms = params["loop_update_rate_ms"]
        req.wheel_max_angular_velocity = params["wheel_max_angular_velocity"]
        req.smoothing_factor = params["smoothing_factor"]
        req.kp = params["kp"]
        req.ki = params["ki"]
        req.kd = params["kd"]

        res = service(req)
        if res.success == False:
            rospy.logwarn("Params already sent.")
            return True
    except:
        rospy.logwarn("Exception occured! Failed to send parameters.")
        return False
    return True

def send_imu_params(namespace, param_service, params):
    full_service_name = "/" + namespace + param_service
    rospy.loginfo("Sending parameters to %s" % (full_service_name))
    rospy.wait_for_service(full_service_name)
    try:
        service = rospy.ServiceProxy(full_service_name, SetImuParams)

        req = SetImuParamsRequest()
        
        req.imu_update_rate_ms = params["imu_update_rate_ms"]
        req.acc_bias = params["acc_bias"]
        req.gyro_bias = params["gyro_bias"]
        req.mag_bias = params["mag_bias"]
        req.mag_scale = params["mag_scale"]
        req.mag_dec = params["mag_dec"]

        res = service(req)
        if res.success == False:
            rospy.logwarn("Params already sent.")
            return True
    except:
        rospy.logwarn("Exception occured! Failed to send parameters.")
        return False
    return True

def send_odom_params(namespace, param_service, params):
    full_service_name = "/" + namespace + param_service
    rospy.loginfo("Sending parameters to %s" % (full_service_name))
    rospy.wait_for_service(full_service_name)
    try:
        service = rospy.ServiceProxy(full_service_name, SetOdomParams)

        req = SetOdomParamsRequest()
        req.wheel_radius = params["wheel_radius"]
        req.distance_between_wheels = params["distance_between_wheels"]
        req.odom_update_rate_ms = params["odom_update_rate_ms"]

        res = service(req)
        if res.success == False:
            rospy.logwarn("Params already sent.")
            return True
    except:
        rospy.logwarn("Exception occured! Failed to send parameters.")
        return False
    return True


rospy.init_node("real_time_param_sender", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
left_motor_param_service_name = rospy.get_param("~left_motor_param_service_name", "/real_time/motors/left/set_params")
right_motor_param_service_name = rospy.get_param("~right_motor_param_service_name", "/real_time/motors/right/set_params")
imu_param_service_name = rospy.get_param("~imu_param_service_name", "/real_time/imu/set_params")
odom_param_service_name = rospy.get_param("~odom_param_service_name", "/real_time/odom/set_params")

left_motor_pid = rospy.get_param("~left_motor_pid", [0,0,0])
right_motor_pid = rospy.get_param("~right_motor_pid", [0,0,0])

left_motor_params = {
    "loop_update_rate_ms": rospy.get_param("~loop_update_rate_ms", 0),
    "wheel_max_angular_velocity": rospy.get_param("~wheel_max_angular_velocity", 0),
    "smoothing_factor": rospy.get_param("~smoothing_factor", 0),
    "kp": left_motor_pid[0],
    "ki": left_motor_pid[1],
    "kd": left_motor_pid[2]
}

right_motor_params = {
    "loop_update_rate_ms": rospy.get_param("~loop_update_rate_ms", 0),
    "wheel_max_angular_velocity": rospy.get_param("~wheel_max_angular_velocity", 0),
    "smoothing_factor": rospy.get_param("~smoothing_factor", 0),
    "kp": right_motor_pid[0],
    "ki": right_motor_pid[1],
    "kd": right_motor_pid[2]
}

imu_params = {
    "imu_update_rate_ms": rospy.get_param("~imu_update_rate_ms", 0),
    "acc_bias": rospy.get_param("~acc_bias", [0,0,0]),
    "gyro_bias": rospy.get_param("~gyro_bias", [0,0,0]),
    "mag_bias": rospy.get_param("~mag_bias", [0,0,0]),
    "mag_scale": rospy.get_param("~mag_scale", [0,0,0]),
    "mag_dec": rospy.get_param("~mag_dec", [0, 0, 0])
}

odom_params = {
    "wheel_radius": rospy.get_param("~wheel_radius", 0),
    "distance_between_wheels": rospy.get_param("~distance_between_wheels", 0),
    "odom_update_rate_ms": rospy.get_param("~odom_update_rate_ms", 0)
}

time.sleep(6)

while(not send_motor_params(statek_name, left_motor_param_service_name, left_motor_params)):
    time.sleep(5)
time.sleep(1)

send_motor_params(statek_name, right_motor_param_service_name, right_motor_params)
time.sleep(1)

send_imu_params(statek_name, imu_param_service_name, imu_params)
time.sleep(1)

send_odom_params(statek_name, odom_param_service_name, odom_params)