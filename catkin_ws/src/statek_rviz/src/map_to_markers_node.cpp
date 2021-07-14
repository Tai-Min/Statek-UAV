#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

namespace
{
    std::string statekName;
    std::string mapTopic;
    std::string mapFrame;

    double mapSizeMeters;
    double cellSizeMeters;
    int mapUpdateRateMs;
    unsigned int numCellsPerRowCol;

    ros::Subscriber mapSub;
    ros::Publisher mapMarkerPublisher;
}

enum CellType
{
    UNKNOWN_CELL = -1,
    FREE_CELL = 0,
    FILLED_GAP = 90,
    OBSTACLE_CELL = 100,
};

double toMeters(unsigned int idx)
{
    double result = idx;
    result -= numCellsPerRowCol / 2.0;
    return result * cellSizeMeters;
}

visualization_msgs::Marker createMarker(
    int id, double x, double y,
    double colorR, double colorG, double colorB, double colorA)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time(0); // Fixes extrapolation into the future if Rviz is run remotely.
    marker.header.frame_id = mapFrame;

    marker.ns = statekName + "/local_map";
    marker.id = id;
    marker.type = 1;
    marker.action = 0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.35 / 2.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = cellSizeMeters;
    marker.scale.y = cellSizeMeters;
    marker.scale.z = 0.35;

    marker.color.r = colorR;
    marker.color.g = colorG;
    marker.color.b = colorB;
    marker.color.a = colorA;

    //marker.lifetime.nsec = 500000000;

    return marker;
}

void publishMarkers(const nav_msgs::OccupancyGrid &mapMsg)
{
    int markerId = 0;
    visualization_msgs::MarkerArray markersMsg;

    visualization_msgs::Marker deleteAll;
    deleteAll.action = 3;
    markersMsg.markers.push_back(deleteAll);
    mapMarkerPublisher.publish(markersMsg);
    markersMsg.markers.clear();

    for (int y = 0; y < numCellsPerRowCol; y++)
    {
        for (int x = 0; x < numCellsPerRowCol; x++)
        {
            if (mapMsg.data[y * numCellsPerRowCol + x] == CellType::OBSTACLE_CELL)
            {
                markersMsg.markers.push_back(createMarker(markerId, toMeters(x), toMeters(y), 0, 0, 0, 1));
                markerId++;
            }
            else if (mapMsg.data[y * numCellsPerRowCol + x] == CellType::FILLED_GAP)
            {
                markersMsg.markers.push_back(createMarker(markerId, toMeters(x), toMeters(y), 0.25, 0.25, 0.25, 1));
                markerId++;
            }
        }
    }

    mapMarkerPublisher.publish(markersMsg);
}

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "map_to_markers");
    ros::NodeHandle nh("~");

    nh.param<std::string>("statek_name", statekName, "statek");

    nh.param<std::string>("map_topic", mapTopic, "/" + statekName + "/map/local_map");
    nh.param<std::string>("map_frame", mapFrame, statekName + "/map/local_map_link");

    nh.param<double>("map_size_meters", mapSizeMeters, 7);
    nh.param<double>("cell_size_meters", cellSizeMeters, 0.1);
    nh.param<int>("map_update_rate_ms", mapUpdateRateMs, 0);

    numCellsPerRowCol = mapSizeMeters / cellSizeMeters;

    mapSub = nh.subscribe(mapTopic, 1, &publishMarkers);
    mapMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/" + statekName + "/map_markers", 1);

    ros::spin();
}