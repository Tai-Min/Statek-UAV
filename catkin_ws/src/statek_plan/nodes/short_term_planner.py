#!/usr/bin/env python
import rospy
from astar import AStar
from statek_map.msg import Graph
from nav_msgs.msg import Path
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

def voronoi_graph_callback(graph, data):
    path_msg = Path()
    path_msg.header.frame_id = data["map_link"]
    path_msg.header.stamp =  rospy.Time.now()

    start = graph.nodes[-1] # Goal is always last.
    end = graph.nodes[-2] # Start is always second from end.

    voronoi_graph_callback.planner.set_graph(graph.nodes)
    path = voronoi_graph_callback.planner.astar(start, end)
    if path:
        for node in path:
            pose = PoseStamped()
            pose.pose.position.x = node.point.x
            pose.pose.position.y = node.point.y
            path_msg.poses.append(pose)
    
    data["path_publisher"].publish(path_msg)

voronoi_graph_callback.planner = Planner()

def main():
    # Init ROS.
    rospy.init_node('short_term_planenr', anonymous=True)
    
    # Get params.
    statek_name = rospy.get_param("~statek_name", "statek")
    voronoi_graph_topic = rospy.get_param("~voronoi_graph_topic", "/" + statek_name + "/map/voronoi_map")
    short_term_path_topic = rospy.get_param("~short_term_path_topic", "/" + statek_name + "/short_term_path")
    local_map_link = rospy.get_param("~local_map_link", statek_name + "/map/local_map_link")

    # Init subscribers and publishers.
    path_publisher = rospy.Publisher(short_term_path_topic, Path, queue_size=1)
    rospy.Subscriber(voronoi_graph_topic, Graph, voronoi_graph_callback, {"map_link": local_map_link, "path_publisher": path_publisher}, queue_size=1)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()