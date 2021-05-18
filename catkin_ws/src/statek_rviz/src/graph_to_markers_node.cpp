#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <statek_map/Graph.h>

namespace
{
    std::string statekName;
    std::string mapTopic;
    std::string mapFrame;

    ros::Subscriber mapSub;
    ros::Publisher mapMarkerPublisher;
}

visualization_msgs::Marker createNodeMarker(
    int id, double x, double y,
    double colorR, double colorG, double colorB, double colorA)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = mapFrame;

    marker.ns = statekName + "/voronoi_map_nodes";
    marker.id = id;
    marker.type = 2; // Sphere.
    marker.action = 0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = colorR;
    marker.color.g = colorG;
    marker.color.b = colorB;
    marker.color.a = colorA;

    marker.lifetime.nsec = 300000000;

    return marker;
}

void publishMarkers(const statek_map::Graph &graphMsg)
{
    int markerId = 0;
    visualization_msgs::MarkerArray markersMsg;

    for (int i = 0; i < graphMsg.nodes.size(); i++)
    {
        markersMsg.markers.push_back(
            createNodeMarker(markerId, graphMsg.nodes[i].point.x, graphMsg.nodes[i].point.y, 0, 1, 0, 1));
        markerId++;
    }

    mapMarkerPublisher.publish(markersMsg);
}

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "graph_to_markers");
    ros::NodeHandle nh("~");

    nh.param<std::string>("statek_name", statekName, "statek");

    nh.param<std::string>("voronoi_map_topic", mapTopic, "/" + statekName + "/map/voronoi_map");
    nh.param<std::string>("voronoi_map_frame", mapFrame, statekName + "/map/local_map_link");

    mapSub = nh.subscribe(mapTopic, 1, &publishMarkers);
    mapMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

    ros::spin();
}