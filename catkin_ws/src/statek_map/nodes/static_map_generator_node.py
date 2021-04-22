#!/usr/bin/env python
import rospy
import numpy as np
import sys
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid

OBSTACLE = 1

map_params = {}
laser_map = []
segmentation_map = []
dynamic_map = []

def to_index(val):
    global map_params
    val /= map_params["cell_size_meters"] # Scaled to indexes in laser_map
    val += map_params["num_cells_per_row_col"] / 2 # Offset added So x and y are from center now
    return int(round(val)) # To closest integer index

def to_meters(val):
    global map_params
    val -= map_params["num_cells_per_row_col"] / 2 # Remove offset
    return val * map_params["cell_size_meters"] # Rescale to meters

def generate_rviz_map_marker(x, y, frame_id, namespace, marker_id, scale, color, lifetime):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = 1
    marker.action = 0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = scale[2] / 2

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    marker.lifetime.secs = lifetime

    return marker

def publish_rviz_map_markers(publisher, map_matrix):
    global map_params

    if map_matrix == []:
        return

    marker_id = 0
    markers = MarkerArray()

    shape = map_matrix.shape 
    for y in range(shape[0]):
        for x in range(shape[1]):
            if map_matrix[y][x] == 1:
                x_meters = to_meters(x)
                y_meters = to_meters(y)

                marker = generate_rviz_map_marker(x_meters, y_meters, map_params["map_frame"], map_params["namespace"] + "_static_map",
                                                marker_id, [map_params["cell_size_meters"], map_params["cell_size_meters"], 0.35],
                                                [1, 0, 0, 1], map_params["update_rate"] * 1.5)
                markers.markers.append(marker)
                marker_id += 1

    publisher.publish(markers)

def publish_final_map(publisher, map_matrix):
    global map_params

    if map_matrix == []:
        return

    map_msg = OccupancyGrid()

    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = map_params["map_frame"]

    map_msg.info.resolution = map_params["cell_size_meters"]
    map_msg.info.width = map_params["num_cells_per_row_col"]
    map_msg.info.height = map_params["num_cells_per_row_col"]
    map_msg.info.origin.position.x -= map_params["cell_size_meters"] * map_params["num_cells_per_row_col"] / 2
    map_msg.info.origin.position.y -= map_params["cell_size_meters"] * map_params["num_cells_per_row_col"] / 2

    map_msg.data = (map_matrix * 100).flatten().astype(int).tolist()

    publisher.publish(map_msg)

def laser_callback(laser_data):
    global laser_map
    global map_params

    laser_map = np.zeros((map_params["num_cells_per_row_col"], map_params["num_cells_per_row_col"]))

    current_angle = laser_data.angle_min
    angle_increment = laser_data.angle_increment
    ranges = laser_data.ranges

    for ray in ranges:
        # ray == 0 means there was no obstacle detection
        if ray == 0:
            current_angle += angle_increment
            continue

        x = to_index(ray * math.cos(current_angle))
        y = to_index(ray * math.sin(current_angle))

        # Position out of bounds
        if x < 0 or y < 0 or x >= map_params["num_cells_per_row_col"] or y >= map_params["num_cells_per_row_col"]:
            current_angle += angle_increment
            continue

        laser_map[y][x] = OBSTACLE
        current_angle += angle_increment

def fuse_maps():
    global map_params
    global laser_map
    global segmentation_map
    global dynamic_map

    return laser_map

def generate_final_map():
    final_map = fuse_maps()

    final_map = close_small_gaps(final_map)

    return final_map 

if __name__ == '__main__':
    rospy.init_node('static_map_generator', anonymous=True)
    

    # Some common stuff
    statek_name = "statek"
    map_frame = statek_name + "/local_map"

    # Map's parameters
    map_size = 7
    cell_size = 0.1
    minimum_gap_size = 0.5

    map_update_rate = 1

    # Topics.
    laser_topic = statek_name + "/laser/scan"
    map_topic = statek_name + "/local_map"

    # Global params
    map_params = {
        "namespace": statek_name,
        "map_frame": map_frame,
        "map_size_meters": map_size,
        "cell_size_meters": cell_size, # Size of one grid square.
        "minimum_gap_size_meters": minimum_gap_size,
        "num_cells_per_row_col": int(map_size / cell_size), # Number of cells in one row or column.
        "update_rate": map_update_rate # In Hz.
    }

    rospy.Subscriber(laser_topic, LaserScan, laser_callback, queue_size=1)

    map_publisher = rospy.Publisher(map_topic, OccupancyGrid, queue_size=1)
    rviz_marker_publisher = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=1)

    rate = rospy.Rate(map_params["update_rate"])
    while not rospy.is_shutdown():
        final_map = generate_final_map()

        publish_final_map(map_publisher, final_map)
        publish_rviz_map_markers(rviz_marker_publisher, final_map)

        rate.sleep()
        
