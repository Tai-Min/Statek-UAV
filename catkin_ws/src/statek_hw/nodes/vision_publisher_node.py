#!/usr/bin/env python
import cv2
import numpy as np
from threading import Thread, Lock
import time
import yaml
import rospy
import rospkg
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
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

class RosCamera:
	def __init__(self, namespace, position, width, height, framerate, flip):
		# settings
		self.cntr = 0 # for header.seq
		self.namespace = namespace
		self.position = position
		self.width = width
		self.height = height

		self.tf_link = self.namespace + "/stereo/" + self.position + "_link"
		self.image_raw_topic = "/" + self.namespace + "/stereo/" + self.position + "/image_raw"
		self.camera_info_topic = "/" + self.namespace + "/stereo/" + self.position + "/camera_info"
		self.camera_info_service = "/" + self.namespace + "/stereo/" + self.position + "/set_camera_info"
		self.camera_param_namespace = "~" + self.namespace + "/camera_info_" + self.position + "/"

		# ros stuff
		self.raw_publisher = rospy.Publisher(self.image_raw_topic, Image, queue_size=10)
		self.info_publisher = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=10)
		self.set_info_service = rospy.Service(self.camera_info_service, SetCameraInfo, self.set_camera_info)
		self.info_msg = self.get_camera_info()
		
		# camera driver
		self.camera = Camera()
		self.camera.open(0 if position == "left" else 1, width, height, framerate, flip)
		if not self.camera.is_opened():
			print("Failed to open /dev/video0")
			exit()

	def __del__(self):
		self.camera.release()

	def get_camera_info(self):
		info = CameraInfo()
		info.header.frame_id = self.tf_link
		info.width = self.width
		info.height = self.height
		info.distortion_model = rospy.get_param(self.camera_param_namespace + "distortion_model", "plump_bob")
		info.D = rospy.get_param(self.camera_param_namespace + "D", [0,0,0,0,0])
		info.K = rospy.get_param(self.camera_param_namespace + "K", [0,0,0,0,0,0,0,0,0])
		info.R = rospy.get_param(self.camera_param_namespace + "R", [0,0,0,0,0,0,0,0,0])
		info.P = rospy.get_param(self.camera_param_namespace + "P", [0,0,0,0,0,0,0,0,0,0,0,0])
		return info

	def set_camera_info(self, info):
		# local save
		self.info_msg.distortion_model = info.camera_info.distortion_model
		self.info_msg.D = info.camera_info.D
		self.info_msg.K = info.camera_info.K
		self.info_msg.R = info.camera_info.R
		self.info_msg.P = info.camera_info.P

		# save to config
		data = dict(
			distortion_model = info.camera_info.distortion_model,
			D = list(info.camera_info.D),
			K = list(info.camera_info.K),
			R = list(info.camera_info.R),
			P = list(info.camera_info.P),
		)
		result = SetCameraInfoResponse()
		try:
			with open(rospkg.RosPack().get_path("statek_config") + "/yaml/camera_info_" + self.position + ".yaml", 'w') as conf:
				yaml.dump(data, conf, default_flow_style=False)
			result.success = True
			result.status_message = "ok"
		except Exception as e:
			result.success = False
			result.status_message = str(e)
		return result

	def publish(self, stamp):
		frame = self.camera.get_frame()

		frame_msg = cv_bridge.cv2_to_imgmsg(frame, "bgr8")
		frame_msg.header.seq = self.cntr
		frame_msg.header.stamp = stamp
		frame_msg.header.frame_id = self.tf_link

		self.info_msg.header.seq = self.cntr
		self.info_msg.header.stamp = stamp

		self.raw_publisher.publish(frame_msg)
		self.info_publisher.publish(self.info_msg)

		self.cntr += 1

rospy.init_node("vision_publisher", anonymous=True)

# ROS settings
statek_name = rospy.get_param("~statek_name", "statek")

# camera settings
config_namespace = "~" + statek_name + "/camera_config/"
camera_width = rospy.get_param(config_namespace + "width", 480)
camera_height = rospy.get_param(config_namespace + "height", 270)
camera_framerate = rospy.get_param(config_namespace + "framerate", 15)
camera_flip = rospy.get_param(config_namespace + "flip", 0)

# rate same as framerate
rate = rospy.Rate(camera_framerate)

cam_center = RosCamera(statek_name, "left", camera_width, camera_height, camera_framerate, camera_flip)
cam_right = RosCamera(statek_name, "right", camera_width, camera_height, camera_framerate, camera_flip)

# cv::Mat to Image msg
cv_bridge = CvBridge()

while not rospy.is_shutdown():

	stamp = rospy.Time.now()
	cam_center.publish(stamp)
	cam_right.publish(stamp)

	rate.sleep()

