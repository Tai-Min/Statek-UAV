#!/usr/bin/env python3
from tensorflow.python.saved_model import tag_constants
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2
import rospy
import rospkg
from sensor_msgs.msg import Image
import tensorflow as tf
import sys
import numpy as np

# Load _get_legs function from petranet's preprocessing.
r = rospkg.RosPack()
path = r.get_path("statek_ml")
path += "/models/lidar_net"
sys.path.insert(1, path)
get_legs = __import__("dataset_processing")
get_legs = get_legs._get_legs

class LegKalman():
    pass

class Leg():
    
    def __init__(self, leg, forget_time, max_distance, process_variance, measurement_variance):
        # @brief Class constructor.
        # @param leg Initial coordinates of the leg in pixel domain.
        # @param forget_time Period of time in which update() should success at least once. 
        # If there was no succesfull update in this time period then alive() will return False
        # and leg should be removed.
        # @param max_distance Maximum allowed euclidean distance in meters between current leg position (from Kalman)
        # and leg candidates coordinates. This distance assumes update time of 1 second.
        # @param process_variance Process variance for Kalman filter.
        # @param measurement_variance Measurement variance for Kalman filter.
        self.position = leg

    def update(self, leg_candidates):
        # @brief Update leg object.
        # @param List of legs detected from PeTraNet prediction. 
        # If the update succeeds then this function will remove most promising 
        # leg candidate.
        # @param dt Time passed since last call to update()
        # @return True on success.
        pass

    def alive(self):
        # @brief Whether this object should be destroyed.
        # @return True if should be destroyed.
        pass

    def position_pixels(self):
        # @brief Get this leg's position (from Kalman filer).
        # @return tuple of (y, x) coordinates in pixel domain.
        pass

    def position_meters(self):
        # @brief Get this leg's position (from Kalman filer).
        # @return tuple of (y, x) coordinates in SI domain.
        pass

    def velocity_meters(self):
        # @brief Get this leg's velocity (from Kalman filer).
        # @return tuple of (y, x) velocities in SI domain.
        pass

def load_network():
    model_path = rospkg.RosPack().get_path("statek_ml")
    model_path += "/models/lidar_net"
    sys.path.insert(1, model_path)

    net = tf.saved_model.load(model_path + "/trained_model_optimized", tags=[tag_constants.SERVING])
    net = net.signatures[tf.compat.v1.saved_model.signature_constants.DEFAULT_SERVING_SIGNATURE_DEF_KEY]
    net = convert_variables_to_constants_v2(net)

    preprocessor = __import__("dataset_processing")
    preprocessor = preprocessor.preprocess_input_sample

    return net, preprocessor

# Can't get cv_bridge to work on python3 and Jeston Nano D:
def msg_to_img(msg):
    frame = np.array(list(msg.data))
    frame = np.reshape(frame, (1, msg.height, msg.width))
    return frame

def prediction_to_cv(prediction):
    prediction = prediction[0].numpy()
    prediction = np.round(prediction)
    prediction *= 255.0
    prediction = prediction.astype(np.uint8)
    prediction = np.squeeze(prediction)
    return prediction

def to_meters(y, x, pixel_height, pixel_width, height, width):
    # @brief convert coordinates of pixel to 
    pass

def scan_callback(msg):
    # Make prediction.
    frame = msg_to_img(msg)
    frame = scan_callback.preprocessor(frame)
    prediction = scan_callback.net(frame)
    prediction = prediction_to_cv(prediction)
    leg_candidates = get_legs(prediction)

    # Update legs.
    for leg in scan_callback.legs:
        leg.update(leg_candidates)

    # Remove dead legs.
    scan_callback.legs = [leg for leg in scan_callback.legs if leg.alive()]

    # Generate new legs from candidates that were not consumed by already
    # existing legs.
    for candidate in leg_candidates:
        scan_callback.legs.append(Leg(candidate, scan_callback.forget_time, scan_callback.max_distance,
        scan_callback.process_variance, scan_callback.measurement_variance))

    msg.data = prediction.flatten().tolist()
    msg.header.stamp = rospy.Time.now()
    scan_callback.publisher.publish(msg)

    print(leg_candidates)


# Limit memory usage as it's shared with CPU.
gpu_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(gpu_devices[0], True)
tf.config.experimental.set_virtual_device_configuration(
            gpu_devices[0],
            [tf.config.experimental.VirtualDeviceConfiguration(
               memory_limit=512)])

# Init ROS.
rospy.init_node('lidar_fconv_dataset_collector')
statek_name = rospy.get_param("~statek_name", "statek")


# Stuff for laser image callback.
scan_callback.publisher = rospy.Publisher("/" + statek_name + "/laser/legs_img", Image, queue_size=1)
scan_callback.legs = []
scan_callback.forget_time = rospy.get_param("~leg_forget_time", "/real_time/motors/left/encoder")
scan_callback.max_distance = rospy.get_param("~candidate_max_distance", "/real_time/motors/left/encoder")
scan_callback.process_variance = rospy.get_param("~leg_process_variance", "/real_time/motors/left/encoder")
scan_callback.measurement_variance = rospy.get_param("~leg_measurement_variance", "/real_time/motors/left/encoder")
scan_callback.net, scan_callback.preprocessor = load_network()

# Init lidar image subscriber.
rospy.Subscriber("/" + statek_name + "/laser/scan_img", Image, scan_callback, queue_size=1)

rospy.spin()