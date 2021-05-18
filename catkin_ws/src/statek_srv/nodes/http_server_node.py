#!/usr/bin/env python
import http.server
import socketserver
import threading
import rospy
import robot_resource.robot_resource as rs
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

class RequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/robotInfo":
            rs.GET_robot_info(self)
        elif self.path == "/robotStatus":
            rs.GET_robot_status(self)
        elif self.path == "/leftCameraFrame":
            rs.GET_left_camera_frame(self)

        #return http.server.SimpleHTTPRequestHandler.do_GET(self)

def main():
    # Init ROS.
    rospy.init_node('http_server', anonymous=True)

    # Get params.
    statek_name = rospy.get_param("~statek_name", "statek")
    left_camera_topic = rospy.get_param("~left_camera_topic", "/" + statek_name + "/stereo/left/image_rect_color")
    gps_topic = rospy.get_param("~gps_topic", "/" + statek_name + "/gps/fix")

    # Set params to robot resource.
    rs.ROS_set_robot_info(statek_name)

    # Init subscribers.
    rospy.Subscriber(left_camera_topic, Image, rs.ROS_camera_frame_callback, queue_size=1)
    rospy.Subscriber(gps_topic, NavSatFix, rs.ROS_gps_fix_callback, queue_size=1)

    PORT = 1339
    handler = RequestHandler
    httpd = socketserver.TCPServer(("", PORT), handler)

    http_thread = threading.Thread(name='http_thread', target=httpd.serve_forever)
    http_thread.start()

    while not rospy.is_shutdown():
        pass

    httpd.shutdown()
    http_thread.join()

if __name__ == '__main__':
    main()