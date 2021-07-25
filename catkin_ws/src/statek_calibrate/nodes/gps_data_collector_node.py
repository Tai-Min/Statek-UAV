#!/usr/bin/env python
import rospy
import rospkg
import time
import sys
from sensor_msgs.msg import NavSatFix
from datetime import datetime

def status_to_string(status):
    if status == -1:
        return "STATUS_NO_FIX"
    elif status == 0:
        return "STATUS_FIX"
    elif status == 1:
        return "STATUS_SBAS_FIX"
    elif status == 2:
        return "STATUS_GBAS_FIX"
    return ""

def gps_callback(gps_data, params):
    data_str = str(datetime.now().strftime("%Y-%m-%d %H:%M:%S")) + ","
    data_str += str(status_to_string(gps_data.status.status)) + "," 
    data_str += str(gps_data.latitude) + "," 
    data_str += str(gps_data.longitude) + "," 
    data_str += str(gps_data.altitude) + "\n"

    with open(params["file_path"], "a") as f:
        f.write(data_str)

rospy.init_node("gps_data_collector_node", anonymous=True)

statek_name = rospy.get_param("~statek_name", "statek")
file_name = rospy.get_param("~file_name", "gps_data")
work_time_seconds = rospy.get_param("~work_time_seconds", 600)
append_header = rospy.get_param("~append_header", True)
gps_topic = rospy.get_param("~gps_topic", "/" + statek_name + "/gps/fix")

r = rospkg.RosPack()
path = r.get_path("statek_calibrate")
conf_name = file_name + ".csv"
full_path = path + "/data/" + conf_name

params = {
    "file_path": full_path
}

if append_header:
    data_str = "timestapm,status,latitute,longitude,altitude\n"
    open(params["file_path"], "w").close() # Clear contents of the file.
    with open(params["file_path"], "a") as f:
        f.write(data_str)

rospy.Subscriber(gps_topic, NavSatFix, gps_callback, params, queue_size=1)

start_stamp = time.time()

while not rospy.is_shutdown():
    rospy.sleep(0.5)
    current_stamp = time.time()
    if current_stamp - start_stamp >= work_time_seconds:
        sys.exit(0)