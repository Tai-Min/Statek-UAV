#!/usr/bin/env python3
from tensorflow.python.saved_model import tag_constants
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2
import rospy
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
import tensorflow as tf
import sys
import numpy as np
import time
import math
import threading

# Load _get_legs function from petranet's preprocessing.
r = rospkg.RosPack()
path = r.get_path("statek_ml")
path += "/models/lidar_net"
sys.path.insert(1, path)
get_legs = __import__("dataset_processing")
get_legs = get_legs._get_legs

data_path = r.get_path("statek_ml") + "/data"

class LegKalman():
    def __init__(self, leg_position, process_variance, measurement_variance):
        # @brief Class constructor.
        # @param leg_position Initial leg position (y, x).
        # @param process_variance Process variance.
        # @param measurement_variance Measurement variance.
    
        # Parameters.
        self.Q = np.array([[process_variance, 0], [0, process_variance]])
        self.R = np.array([[measurement_variance, 0],[0, measurement_variance]])

        # Initial conditions.
        y = leg_position[0]
        x = leg_position[1]
        self.a_posteriori_xhat = np.array([[y], [x]])
        self.a_posteriori_P = self.Q

    def update(self, velocity_measurement, position_measurement, dt):
        # @brief Update filter.
        # @param velocity_measurement Velocity measurement based on current position measurement
        # and previous estimate (vy, vx).
        # @param position_measurement Current position measurement.
        # @param Time passed since last update.
        # @return Position estimate (y, x).

        vy = velocity_measurement[0]
        vx = velocity_measurement[1]
        
        y = position_measurement[0]
        x = position_measurement[1]
        u = np.array([[vy], [vx]])
        z = np.array([[y], [x]])
        B = np.array([[dt, 0], [0, dt]])

        # Predict.
        a_priori_xhat = self.a_posteriori_xhat + B @ u
        a_priori_P = self.a_posteriori_P + self.Q

        # Update.
        innovation = z - a_priori_xhat
        innovation_covariance = a_priori_P + self.R
        K = a_priori_P @ np.transpose(innovation_covariance)
        self.a_posteriori_xhat = a_priori_xhat + K @ innovation
        self.a_posteriori_P = (np.identity(2) - K) @ a_priori_P

        y = self.a_posteriori_xhat[0]
        x = self.a_posteriori_xhat[1]

        return (y[0], x[0])

class Leg():
    
    def __init__(self, leg, forget_time, max_distance, process_variance, measurement_variance):
        # @brief Class constructor.
        # @param leg Initial coordinates of the leg in meters [y, x].
        # @param forget_time Period of time in seconds in which update() should success at least once. 
        # If there was no succesfull update in this time period then alive() will return False
        # and leg should be removed.
        # @param max_distance Maximum allowed euclidean distance in meters between current leg position (from Kalman)
        # and leg candidates coordinates. This distance assumes update time of 1 second.
        # @param process_variance Process variance for Kalman filter.
        # @param measurement_variance Measurement variance for Kalman filter.

        # Params.
        self.forget_time = forget_time
        self.max_distance = max_distance

        # First update.
        self.filter = LegKalman(leg, process_variance, measurement_variance)
        self.last_update = time.time()
        self.estimated_velocity = (0,0)
        self.estimated_position = leg

    def _get_distance(self, candidate):
        # @brief Get candidate's distance from current leg's estimated position.
        # @param candidate Candidate to check (y, x).
        # @return Distance from candidate to current's position.

        return math.sqrt((candidate[0] - self.estimated_position[0])**2 + (candidate[1] - self.estimated_position[1])**2)

    def _find_candidate(self, leg_candidates):
        # @brief Find most promising leg candidate.
        # @param leg_candidates List od leg candidates [(y,x), (y,x), ...].
        # @return Tuple in which the first element is found distance and second is it's index.

        distances = [self._get_distance(candidate) for candidate in leg_candidates]
        min_distance = min(distances)
        return (min_distance, distances.index(min_distance))


    def update(self, leg_candidates):
        # @brief Update leg object.
        # @param List of legs detected from PeTraNet prediction [(y, x), (y, x), ...]. 
        # If the update succeeds then this function will remove most promising 
        # leg candidate.
        # @param dt Time passed since last call to update()
        # @return True on success.

        if len(leg_candidates) == 0:
            return leg_candidates

        # Check whether any candidate is in range.
        candidate_distance, candidate_index = self._find_candidate(leg_candidates)
        if candidate_distance > self.max_distance:
            return leg_candidates

        measured_position = tuple(leg_candidates[candidate_index])

        # A priori velocity.
        passed = time.time() - self.last_update
        if passed == 0:
            return leg_candidates

        measured_velocity_y = (measured_position[0] - self.estimated_position[0]) / passed
        measured_velocity_x = (measured_position[1] - self.estimated_position[1]) / passed
        measured_velocity = (measured_velocity_y, measured_velocity_x)
        
        # Position estimation.
        previous_position_estimate = self.estimated_position
        self.estimated_position = self.filter.update(measured_velocity, measured_position, passed)

        # A posteriori velocity.
        estimated_velocity_y = (self.estimated_position[0] - previous_position_estimate[0]) / passed
        estimated_velocity_x = (self.estimated_position[1] - previous_position_estimate[1]) / passed
        self.estimated_velocity = (estimated_velocity_y, estimated_velocity_x)

        del leg_candidates[candidate_index]
        self.last_update = time.time()

        return leg_candidates

    def alive(self):
        # @brief Whether this object should be destroyed.
        # @return True if leg is still alive.

        return (time.time() - self.last_update) < self.forget_time

    def position(self):
        # @brief Get this leg's estimated position.
        # @return tuple of (y, x) coordinates.

        return self.estimated_position

    def velocity(self):
        # @brief Get this leg's estimated velocity.
        # @return tuple of (y, x) velocities.

        return self.estimated_velocity

def load_network():
    # @brief Load network and it's preprocessing function.
    # @return (network model, preprocessing function).

    model_path = rospkg.RosPack().get_path("statek_ml")
    model_path += "/models/lidar_net"
    sys.path.insert(1, model_path)

    rospy.logwarn("Loading model...")
    net = tf.saved_model.load(model_path + "/trained_model_optimized", tags=[tag_constants.SERVING])
    rospy.logwarn("Getting signatures...")
    net = net.signatures[tf.compat.v1.saved_model.signature_constants.DEFAULT_SERVING_SIGNATURE_DEF_KEY]
    rospy.logwarn("Converting to constants...")
    net = convert_variables_to_constants_v2(net)
    rospy.logwarn("Done!")

    preprocessor = __import__("dataset_processing")
    preprocessor = preprocessor.preprocess_input_sample

    return (net, preprocessor)

# Can't get cv_bridge to work on python3 and Jeston Nano D:
def msg_to_img(msg):
    # @brief Convert message to image suitable for preprocessing.
    # @param msg Message to convert.
    # @return Converted message.

    frame = np.array(list(msg.data))
    frame = np.reshape(frame, (1, msg.height, msg.width))
    return frame

def prediction_to_cv(prediction):
    # @brief Convert prediction (0 - 1 float) to cv matrix (0 - 255 uint8).
    # @param prediction Prediction to convert.
    # @return Converted prediction.

    prediction = prediction[0].numpy()
    prediction = np.round(prediction)
    prediction *= 255.0
    prediction = prediction.astype(np.uint8)
    prediction = np.squeeze(prediction)
    return prediction

def to_meters(leg, height_pixels, width_pixels, height, width):
    # @brief convert coordinates from pixel to meters.
    # @param leg Leg to convert [y, x].
    # @param height_pixels Image's height in pixels.
    # @param width_pixels Image's width in pixels.
    # @param height Image's height in meters.
    # @param width Image's width in meters.

    y = float(leg[0])
    x = float(leg[1])

    y -= height_pixels / 2.0
    x -= width_pixels / 2.0

    y *= (height / height_pixels)
    x *= (width / width_pixels)

    return [y, x]

def to_meters_arr(legs, height_pixels, width_pixels, height, width):
    return [to_meters(leg, height_pixels, width_pixels, height, width) for leg in legs]

def get_marker(position, id):
    marker = Marker()
    marker.header.frame_id = "statek/laser/laser_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "legs"
    marker.id = id
    marker.action = Marker.ADD
    marker.type = Marker.SPHERE
    marker.lifetime.secs = 3
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = 0.15
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1
    marker.color.r = 1
    marker.color.b = 0
    marker.color.g = 1
    return marker

def scan_callback(new_msg):
    global msg, msg_lock, msg_arrived
    with msg_lock:
        msg = new_msg
        msg_arrived = True

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


# Stuff for .
publisher = rospy.Publisher("/" + statek_name + "/laser/hoomans", MarkerArray, queue_size=1)
msg = None
msg_lock = threading.Lock()
msg_arrived = False
legs = []
width_meters = rospy.get_param("~width_meters", 5.12)
height_meters = rospy.get_param("~height_meters", 5.12)
width_pixels = rospy.get_param("~width_pixels", 256)
height_pixels = rospy.get_param("~height_pixels", 256)
fps = rospy.get_param("~fps", 15)
forget_time = rospy.get_param("~leg_forget_time", 2)
max_distance = rospy.get_param("~candidate_max_distance", 1.0)
process_variance = rospy.get_param("~leg_process_variance", 0.01)
measurement_variance = rospy.get_param("~leg_measurement_variance", 0.01)
net, preprocessor = load_network()

# Init lidar image subscriber.
rospy.Subscriber("/" + statek_name + "/laser/scan_img", Image, scan_callback, queue_size=1, buff_size=65536*2)

rate = rospy.Rate(fps)
while not rospy.is_shutdown():
    with msg_lock:
        if msg_arrived == False:
            rospy.logwarn("Ignored :(")
            rate.sleep()
            continue

    rospy.logwarn("Arrived!")

    # Preprocess data.
    with msg_lock:
        frame = msg_to_img(msg)
        msg_arrived = False
    frame = preprocessor(frame)

    # Make prediction.
    prediction = net(frame)
    prediction = prediction_to_cv(prediction)

    # Extract legs.
    leg_candidates = get_legs(prediction)
    #leg_candidates = [[20,10],[100,120], [25,33]]
    if len(leg_candidates) == 0:
        rate.sleep()

    leg_candidates = to_meters_arr(leg_candidates,
                                    height_pixels,width_pixels,
                                    height_meters, width_meters)

    # Update legs.
    cntr = 0
    markers = MarkerArray()
    for leg in legs:
        leg_candidates = leg.update(leg_candidates)
        markers.markers.append(get_marker(leg.position(), cntr))
        cntr+=1
        
    publisher.publish(markers)
    rospy.logwarn("pub!")

    # Remove dead legs.
    legs = [leg for leg in legs if leg.alive()]

    # Generate new legs from candidates that were not consumed by already
    # existing legs.
    for candidate in leg_candidates:
        legs.append(Leg(candidate, forget_time, max_distance,
        process_variance, measurement_variance))

    rate.sleep()