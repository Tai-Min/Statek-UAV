#!/usr/bin/env python
import json
import cv2
from threading import Thread, Lock
from cv_bridge import CvBridge
import math

_statek_name = ""

_gps_mutex = Lock()
_gps_location = {
    "longitude" : False,
    "latitude"  : False
}

_robot_battery = "Not tracked"
_robot_status = "Idle"

_camera_mutex = Lock()
_cv_bridge = CvBridge()
_camera_frame = 0

def ROS_set_robot_info(statek_name):
    global _statek_name
    _statek_name = statek_name

def _dd_to_dms(val):
    if math.isnan(val):
        return False

    degree = int(val)
    minute = int((val - degree) * 60)
    second = ((val - degree) * 60 - minute) * 60
    return {
        "degree": degree,
        "minute": minute,
        "second": second
    }

def ROS_gps_fix_callback(fix):
    _gps_mutex.acquire()
    _gps_location["longitude"] = _dd_to_dms(fix.longitude)
    _gps_location["latitude"] = _dd_to_dms(fix.latitude)
    _gps_mutex.release()

def ROS_camera_frame_callback(frame):
    global _cv_bridge, _camera_mutex, _camera_frame
    _camera_mutex.acquire()
    _camera_frame = _cv_bridge.imgmsg_to_cv2(frame, "bgr8")
    _camera_mutex.release()

def GET_robot_info(obj):
    global _statek_name

    _gps_mutex.acquire()
    res = {
        "uavs":[{
            "id": 1,
            "name": _statek_name,
            "icon": "car",
            "movementGraph": "nav_graph.json"
        }]
    }
    _gps_mutex.release()

    obj.send_response(200)
    obj.send_header("Content-type", "application/json")
    obj.send_header('Access-Control-Allow-Origin', '*')
    obj.end_headers()

    obj.wfile.write(bytes(json.dumps(res)))

def GET_robot_status(obj):
    global _gps_location, _robot_battery, _robot_status
    res = {
        "uavs":[{
            "id": 1,
            "battery": _robot_battery,
            "location" : _gps_location,
            "status" : _robot_status
        }]
    }

    obj.send_response(200)
    obj.send_header("Content-type", "application/json")
    obj.send_header('Access-Control-Allow-Origin', '*')
    obj.end_headers()
    obj.wfile.write(bytes(json.dumps(res)))

def GET_left_camera_frame(obj):
    global _camera_mutex, _camera_frame

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]

    _camera_mutex.acquire()
    retval, res = cv2.imencode(".jpeg", _camera_frame, encode_param)
    _camera_mutex.release()

    obj.send_response(200)
    obj.send_header("Content-type", "image/jpeg")
    obj.send_header('Access-Control-Allow-Origin', '*')
    obj.end_headers()
    obj.wfile.write(res.tobytes())