#!/usr/bin/env python

# DO NOT USE
# Does not work as good as I tought :(

import rospy
import tf2_ros
import tf2_geometry_msgs
from astar import AStar
from statek_map.msg import Graph
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class Planner(AStar):
    # Radius of circle around some point
    # in which every point is considered as "close" to this point.
    closest_max_distance = 0.5 

    # Favor distance between two points that are close to points in self.previous_path
    # by decreasing it making it better looking than it really is.
    distance_favor = 0.1

    def set_graph(self, graph):
        self.graph = graph

    def set_previous_path(self, p):
        self.previous_path = p

    def closest_previous_node(self, node):
        min_dist = float("inf")
        found_node = None

        # self.previous_path might not exist.
        try:
            for previous_node in self.previous_path:
                dist = self.distance_between_nodes(node, previous_node)
                if(dist < min_dist):
                    min_dist = dist
                    found_node = previous_node
        except AttributeError:
            pass

        return [found_node, min_dist]

    # Compute close to real cost from start_node to goal using previous map as reference.
    def previous_path_cost(self, start_node):
        previous_node = None
        total_cost = 0

        for node in self.previous_path:
            if node == start_node:
                previous_node = node
            elif previous_node:
                total_cost += self.distance_between_nodes(previous_node, node)
                previous_node = node

        return total_cost

    def heuristic_cost_estimate(self, node, goal):
        [closest_node, node_dist] = self.closest_previous_node(node)

        if node_dist < self.closest_max_distance:
            return self.distance_between_nodes(closest_node, goal) * self.distance_favor
        else:
            return self.distance_between_nodes(node, goal)

    # Compute true distance between two nodes.
    def distance_between_nodes(self, n1, n2):
        x1 = n1.point.x
        y1 = n1.point.y

        x2 = n2.point.x
        y2 = n2.point.y

        return math.hypot(x2 - x1, y2 - y1)

    # Naming necessary for abstract method.
    # Compute distance between two nodes*. 
    # *If both nodes were present in self.previous_path
    # then compute their distance using favor mechanism, 
    # basically decreasing their distance 
    # and making it look better than it really is.
    def distance_between(self, n1, n2):
        [closest_n1, n1_dist] = self.closest_previous_node(n1)
        [closest_n2, n2_dist] = self.closest_previous_node(n2)

        if n1_dist < self.closest_max_distance and n2_dist < self.closest_max_distance:
            return self.distance_between_nodes(closest_n1, closest_n2) * self.distance_favor
        else:
            return self.distance_between_nodes(n1, n2)

    def neighbors(self, node):
        return [self.graph[neighbor] for neighbor in node.neighbors]

tf_buffer =  None
tf_listener =  None
planner = Planner()

def voronoi_graph_callback(graph, data):
    global tf_buffer, tf_listener, planner

    path_msg = Path()
    path_msg.header.frame_id = data["earth_link"]
    path_msg.header.stamp =  rospy.Time.now()

    # Start is last so there is no goal.
    if graph.nodes[-1].isStart:
        return

    start = graph.nodes[-1] # Goal is always last.
    end = graph.nodes[-2] # Start is always second from end.

    voronoi_graph_callback.planner.set_graph(graph.nodes)
    path = voronoi_graph_callback.planner.astar(start, end)

    path_length = 0

    if path:
        path = list(path)
        previous_x = 0
        previous_y = 0
        voronoi_graph_callback.planner.set_previous_path(path)

        for node in path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = data["map_link"]
            pose.pose.position.x = node.point.x
            pose.pose.position.y = node.point.y

            # Transform to earth link.
            transform = tf_buffer.lookup_transform(data["earth_link"],
                                       pose.header.frame_id,
                                       rospy.Time(0),
                                       rospy.Duration(1.0))
            pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

            path_msg.poses.insert(0, pose) # Insert in reverse order so path starts from start node.

            path_length += math.hypot(node.point.x - previous_x, node.point.y - previous_y)
            previous_x = node.point.x
            previous_y = node.point.y
    
    # Publish empty path if it's too short.
    if path_length < data["minimum_path_length"]:
        path_msg.poses[:] = []

    data["path_publisher"].publish(path_msg)

voronoi_graph_callback.planner = Planner()

def main():
    global tf_buffer, tf_listener

    # Init ROS.
    rospy.init_node('short_term_planenr', anonymous=True)
    tf_buffer =  tf2_ros.Buffer()
    tf_listener =  tf2_ros.TransformListener(tf_buffer)

    # Get params.
    statek_name = rospy.get_param("~statek_name", "statek")
    voronoi_graph_topic = rospy.get_param("~voronoi_graph_topic", "/" + statek_name + "/map/voronoi_map")
    short_term_path_topic = rospy.get_param("~short_term_path_topic", "/" + statek_name + "/short_term_path")
    local_map_link = rospy.get_param("~local_map_link", statek_name + "/map/local_map_link")
    earth_link = rospy.get_param("~local_map_link", statek_name + "/earth")
    minimum_path_length = rospy.get_param("~minimum_path_length", 0.5)

    # Init subscribers and publishers.
    path_publisher = rospy.Publisher(short_term_path_topic, Path, queue_size=1)
    params = {"map_link": local_map_link,
              "earth_link": earth_link,
              "path_publisher": path_publisher,
              "minimum_path_length": minimum_path_length}
    rospy.Subscriber(voronoi_graph_topic, Graph, voronoi_graph_callback, params, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()