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
    marker.header.stamp = ros::Time(0); // Fixes extrapolation into the future if Rviz is run remotely.
    marker.header.frame_id = mapFrame;

    marker.ns = statekName + "/voronoi_map_nodes";
    marker.id = id;
    marker.type = 2; // Sphere.
    marker.action = 0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = colorR;
    marker.color.g = colorG;
    marker.color.b = colorB;
    marker.color.a = colorA;

    marker.lifetime.nsec = 500000000;

    return marker;
}

visualization_msgs::Marker createPathMarker(
    int id, double x0, double y0, double x1, double y1,
    double colorR, double colorG, double colorB, double colorA)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time(0); // Fixes extrapolation into the future if Rviz is run remotely.
    marker.header.frame_id = mapFrame;

    marker.ns = statekName + "/voronoi_map_nodes";
    marker.id = id;
    marker.type = 4; // Line strip.
    marker.action = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p0, p1;

    p0.x = x0;
    p0.y = y0;

    p1.x = x1;
    p1.y = y1;

    marker.points.push_back(p0);
    marker.points.push_back(p1);

    marker.scale.x = 0.01;

    marker.color.r = colorR;
    marker.color.g = colorG;
    marker.color.b = colorB;
    marker.color.a = colorA;

    marker.lifetime.nsec = 500000000;

    return marker;
}

void publishMarkers(const statek_map::Graph &graphMsg)
{
    int markerId = 0;
    visualization_msgs::MarkerArray markersMsg;

    int cntr = 0;
    for (auto node : graphMsg.nodes)
    {
        double r = 0;
        double g = 0;
        double b = 0;
        if (node.isGoal || node.isStart)
            b = 1;
        else
            g = 1;

        markersMsg.markers.push_back(createNodeMarker(markerId, node.point.x, node.point.y, r, g, b, 1));

        markerId++;

        for (auto neighborNodeIdx : node.neighbors)
        {
            if (neighborNodeIdx < cntr)
                continue;

            double x0, y0, x1, y1;
            x0 = node.point.x;
            y0 = node.point.y;
            x1 = graphMsg.nodes[neighborNodeIdx].point.x;
            y1 = graphMsg.nodes[neighborNodeIdx].point.y;

            markersMsg.markers.push_back(createPathMarker(markerId, x0, y0, x1, y1, 0, 1, 0, 1));
            markerId++;
        }
        cntr++;
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