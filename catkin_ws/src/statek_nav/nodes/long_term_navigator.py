#!/usr/bin/env python
import rospy
import threading
import tf
import math
import time
from statek_nav.msg import LongTermPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

x = 0
y = 1
z = 2

def wait_goal_callback():
    global goal_arrived, goal_callback_lock

    time.sleep(0.5)

    while True:
        with goal_callback_lock:
            if goal_arrived:
                goal_arrived = False
                return

            time.sleep(0.1)

def goal_callback(new_goal):
    global goal, goal_arrived

    with goal_callback_lock:
        goal = new_goal.pose.position
        goal_arrived = True

def path_callback(new_path):
    global path_lock, path_arrived, path

    with path_lock:
        path = new_path
        path_arrived = True

goal_callback_lock = threading.Lock()
goal_arrived = False
goal = None

path_lock = threading.Lock()
path = None
path_arrived = False
path_counter = 0
path_repeat_count = 0

rospy.init_node('lidar_fconv_dataset_collector')
statek_name = rospy.get_param("~statek_name", "statek")

rospy.Subscriber("/" + statek_name + "/short_term_goal",
                     PoseStamped, goal_callback, queue_size=1)

rospy.Subscriber("/" + statek_name + "/long_term_path",
                     LongTermPath, path_callback, queue_size=1)

goal_gps_publisher = rospy.Publisher("/" + statek_name + "/short_term_goal_gps", NavSatFix, queue_size=1)

rate = rospy.Rate(10)
listener = tf.TransformListener()
while not rospy.is_shutdown():
    with path_lock:
        # Some new path arrived so publish first set of coordinates as goal.
        if path_arrived:
            rospy.logwarn("New path registered.")
            path_counter = 0
            path_repeat_count = 0
            path_arrived = False
            goal_gps_publisher.publish(path.coordinates[path_counter])
            wait_goal_callback()

        # Get position of the vehicle in earth's frame.
        try:
            (trans_footprint, _) = listener.lookupTransform(statek_name + "/earth", statek_name + "/base_footprint", rospy.Time(0))
        except Exception as e:
            rate.sleep()
            continue

        # No goal set yet.
        with goal_callback_lock:
            if goal == None:
                rate.sleep()
                continue

        if path == None:
            rate.sleep()
            continue

        with goal_callback_lock:
            dist = math.hypot(trans_footprint[x] - goal.x, trans_footprint[y] - goal.y)

        # Platform is close enough to current goal.
        if dist < 1.5:

            rospy.logwarn("Checkpoint achieved.")
            path_counter += 1

            # Vehicle finished whole path.
            if path_counter >= len(path.coordinates):

                rospy.logwarn("Path completed.")
                path_repeat_count += 1

                if path.repeat and (path_repeat_count < path.repeat_count or path.repeat_count == 0):

                    if path.repeat_count == 0:
                        rospy.logwarn("Repeating.")

                    else:
                        rospy.logwarn("Finished %d of %d repeats." % (path_repeat_count, path.repeat_count))

                    path_counter = 0

                else:
                    goal = None # Finish current path.
                    continue

            goal_gps_publisher.publish(path.coordinates[path_counter])
            wait_goal_callback()
            
    rate.sleep()