#!/usr/bin/env python
import cv2
import numpy as np
from threading import Thread, Lock
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class Camera:

	def __init__(self):
		# set up stop flag
		self.frame_lock = Lock()
		self.stop_flag = False
		self.stop_lock = Lock()

	def __del__(self):
		try:
			self.close()
		except:
			pass	

	def pipeline(self, dev_id, width, height, framerate, flip):
		return "nvarguscamerasrc sensor_id={dev_id} ! "\
        "video/x-raw(memory:NVMM), "\
        "width=(int){width}, height={height}, "\
        "format=(string)NV12, framerate=(fraction){framerate}/1 ! "\
        "nvvidconv flip-method={flip} ! "\
        "video/x-raw, width={width}, height=(int){height}, format=(string)BGRx ! "\
        "videoconvert ! "\
        "video/x-raw, format=(string)BGR ! appsink".format(dev_id=dev_id, width=width, height=height, framerate=framerate, flip=flip)

	def open(self, dev_id, width, height, framerate, flip):
		# close in case it was already open
		try:
			self.close()
		except: 
			pass

		self.cam = cv2.VideoCapture(self.pipeline(dev_id, width, height, framerate, flip), cv2.CAP_GSTREAMER)

		if self.cam.isOpened():
			self.__start()
		
	def close(self):
		# stop camera thread
		self.__stop()

		# release device
		self.cam.release()

	def is_opened(self):
		return self.cam.isOpened()

	def __start(self):
		# uncheck stop flag
		self.stop_lock.acquire()
		self.stop_flag = False
		self.stop_lock.release()

		# create and start thread
		self.cam_thread = Thread(target=self.__thread_fcn)
		self.cam_thread.daemon = True
		self.cam_thread.start()

		# wait for the first frame
		while True:
			# check if first frame already exists
			self.frame_lock.acquire()
			if hasattr(self, 'frame'):	
				if self.frame is not None:
					self.frame_lock.release()
					break
			self.frame_lock.release()
			
			time.sleep(0.0167)

			
	def __stop(self):
		# tell the thread to stop
		self.stop_lock.acquire()
		self.stop_flag = True
		self.stop_lock.release()

		# wait for thread to end
		self.cam_thread.join()

	def __thread_fcn(self):
		
		while True:
			# check for stop
			self.stop_lock.acquire()
			if self.stop_flag is True:
				self.stop_lock.release()
				break
			self.stop_lock.release()

			# get frame
			self.frame_lock.acquire()
			ret, self.frame = self.cam.read()
			self.frame_lock.release()

			time.sleep(0.0167)

	def get_frame(self):
		# return the frame
		with self.frame_lock:
			return self.frame

def get_camera_info(namespace, tf_link):
	info = CameraInfo()
	info.header.frame_id = tf_link
	info.distortion_model = rospy.get_param("~" + namespace + "distortion_model")
	info.D = rospy.get_param("~" + namespace + "D")
	info.K = rospy.get_param("~" + namespace + "K")
	info.R = rospy.get_param("~" + namespace + "R")
	info.P = rospy.get_param("~" + namespace + "P")
	return info

class RosCamera:
	def __init__(self, position, width, height, framerate, flip):
		pass

	def __del__(self):
		pass

def set_camera_info(info, camera):
	pass

rospy.init_node("vision_publisher", anonymous=True)

# ROS settings
statek_name = rospy.get_param("~statek_name", "statek")

# camera settings
camera_width = rospy.get_param("~camera_width", 480)
camera_height = rospy.get_param("~camera_height", 270)
camera_framerate = rospy.get_param("~camera_framerate", 30)
camera_flip = rospy.get_param("~camera_flip", 0)

# camera info
camera_info_center_msg = get_camera_info(statek_name + "/camera_center/", statek_name + "/center_camera_link")
camera_info_right_msg = get_camera_info(statek_name + "/camera_right/", statek_name + "/right_camera_link")

# rate same as framerate
rate = rospy.Rate(camera_framerate)

# image_raw publishers
center_camera_raw_publisher = rospy.Publisher("/" + statek_name + "/camera_center/image_raw", Image, queue_size=10)
right_camera_raw_publisher = rospy.Publisher("/" + statek_name + "/camera_right/image_raw", Image, queue_size=10)

# camera_info publishers
center_camera_info_publisher = rospy.Publisher("/" + statek_name + "/camera_center/camera_info", CameraInfo, queue_size=10)
right_camera_info_publisher = rospy.Publisher("/" + statek_name + "/camera_right/camera_info", CameraInfo, queue_size=10)

# camera info services
center_camera_info_service = rospy.Service("/" + statek_name + "/set_center_camera_info", CameraInfo, lambda info: set_camera_info(info, "center"))
right_camera_info_service = rospy.Service("/" + statek_name + "/set_right_camera_info", CameraInfo, lambda info: set_camera_info(info, "right"))

# center camera driver
cam_center = Camera()
cam_center.open(0, camera_width, camera_height, camera_framerate, camera_flip)
if not cam_center.is_opened():
	print("Failed to open /dev/video0")
	exit()

# right camera driver
cam_right = Camera()
cam_right.open(1, camera_width, camera_height, camera_framerate, camera_flip)
if not cam_right.is_opened():
	print("Failed to open /dev/video1")
	exit()

# cv::Mat to Image msg
cv_bridge = CvBridge()

# header seq
cntr = 0
while not rospy.is_shutdown():
	frame_center = cam_center.get_frame()
	frame_right = cam_right.get_frame()

	# center camera stuff
	frame_center_msg = cv_bridge.cv2_to_imgmsg(frame_center, "bgr8")
	frame_center_msg.header.seq = cntr
	frame_center_msg.header.stamp = rospy.Time.now()
	frame_center_msg.header.frame_id = statek_name + "/center_camera_link"

	camera_info_center_msg.header.seq = cntr
	camera_info_center_msg.header.stamp = frame_center_msg.header.stamp

	# right camera stuff
	frame_right_msg = cv_bridge.cv2_to_imgmsg(frame_right, "bgr8")
	frame_right_msg.header.seq = cntr
	frame_right_msg.header.stamp = frame_center_msg.header.stamp
	frame_right_msg.header.frame_id = statek_name + "/right_camera_link"

	camera_info_right_msg.header.seq = cntr
	camera_info_right_msg.header.stamp = frame_right_msg.header.stamp

	# publish
	center_camera_raw_publisher.publish(frame_center_msg)
	center_camera_info_publisher.publish(camera_info_center_msg)

	right_camera_raw_publisher.publish(frame_right_msg)
	right_camera_info_publisher.publish(camera_info_right_msg)

	cntr += 1

	rate.sleep()
	

cam0.close()
cam1.close()
cv2.destroyAllWindows()

