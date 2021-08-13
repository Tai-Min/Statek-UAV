#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from astar import AStar
from statek_map.msg import Graph
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import math

class Planner(AStar):
    def set_graph(self, graph):
        self.graph = graph

    def heuristic_cost_estimate(self, n1, n2):
        return self.distance_between(n1, n2)

    def distance_between(self, n1, n2):
        x1 = n1.point.x
        y1 = n1.point.y

        x2 = n2.point.x
        y2 = n2.point.y

        return math.hypot(x2 - x1, y2 - y1)

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

    planner.set_graph(graph.nodes)
    path = planner.astar(start, end)

    path_length = 0

    if path:
        previous_x = 0
        previous_y = 0

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
    minimum_path_length = rospy.get_param("~minimum_path_length", 1.0)

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