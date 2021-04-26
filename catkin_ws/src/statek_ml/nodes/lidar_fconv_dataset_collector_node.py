#!/usr/bin/env python
from numpy.lib.npyio import save
import rospy
import rospkg
import cv2
import time
import numpy as np
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os


def on_new_image(msg):
    global frame_msg
    frame_msg = msg


def create_input_label(frame):
    img_label = np.zeros(frame.shape, frame.dtype)

    cv2.namedWindow("Select output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Select output", (800, 800))
    rois = cv2.selectROIs("Select output", frame, False)

    for roi in rois:
        x1 = roi[0]
        y1 = roi[1]
        x2 = roi[2]
        y2 = roi[3]
        img_label[y1:y1+y2, x1:x1+x2] = frame[y1:y1+y2, x1:x1+x2]

    cv2.namedWindow("Label", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Label", (800, 800))
    cv2.imshow("Label", img_label)
    cv2.waitKey(10)

    time.sleep(1.5)

    cv2.destroyWindow("Select output")
    cv2.destroyWindow("Label")

    return img_label


def save_sample(input_img, output_img, input_folder, output_folder):
    ok = False
    input_path = input_folder + "/" + str(save_sample.cntr) + ".jpg"

    # Find free filename.
    while not ok:
        if os.path.isfile(input_path):
            save_sample.cntr += 1
            input_path = input_folder + "/" + str(save_sample.cntr) + ".jpg"
        else:
            ok = True

    # Generate path for output image too.
    output_path = output_folder + "/" + str(save_sample.cntr) + ".jpg"

    # Save.
    cv2.imwrite(input_path, input_img)
    cv2.imwrite(output_path, output_img)

    # File written so increase the counter.
    save_sample.cntr += 1


save_sample.cntr = 1

# Init ROS.
rospy.init_node('lidar_fconv_dataset_collector')

# Load params.
statek_name = rospy.get_param("~statek_name", "statek")
dataset_folder = rospy.get_param(
    "~dataset_folder", "/home/" + os.environ.get("USER") + "/datasets/lidar_dataset")
dataset_inputs = dataset_folder + "/inputs"
dataset_outputs = dataset_folder + "/outputs"

# Init lidar image subscriber.
rospy.Subscriber("/" + statek_name + "/laser/scan_img", Image, on_new_image)

# Create folders for dataset if necessary.
try:
    os.makedirs(dataset_inputs)
    os.makedirs(dataset_outputs)
except:
    pass

frame_msg = []
bridge = CvBridge()

cv2.namedWindow("Lidar live", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Lidar live", (800, 800))

print("Press space to assign label or ESC to exit.")
while True:
    if frame_msg != []:
        frame = bridge.imgmsg_to_cv2(frame_msg, desired_encoding='passthrough')
        cv2.imshow("Lidar live", frame)

    key = cv2.waitKey(33)

    if key == ord(' '):
        output = create_input_label(frame)
        save_sample(frame, output, dataset_inputs, dataset_outputs)
    if key == 27:  # ESC
        cv2.destroyWindow("Lidar live")
        sys.exit(0)