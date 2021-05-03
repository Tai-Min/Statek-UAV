#include <ros/ros.h>
#include "../include/voronoi_map.hpp"

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "local_mapper");
    ros::NodeHandle nh("~");

    // Get all the params.
    std::string statekName;
    nh.param<std::string>("statek_name", statekName, "statek");

    std::string voronoiMapTopic = "/" + statekName + "/map/voronoi_map";
    std::string localMapTopic = "/" + statekName + "/map/local_map";
    std::string localMapFrame = statekName + "/map/local_map";

    double mapSizeMeters;
    double cellSizeMeters;
    double minimumGapSizeMeters;

    nh.param<double>("map_size_meters", mapSizeMeters, 7);
    nh.param<double>("cell_size_meters", cellSizeMeters, 0.1);
    nh.param<double>("minimum_gap_size_meters", minimumGapSizeMeters, 1.1);

    unsigned int numCellsPerRowCol = mapSizeMeters / cellSizeMeters;

    int mapUpdateRateMs;
    nh.param<int>("map_update_rate_ms", mapUpdateRateMs, 0);

    // Save parameters so all mapping objects can access them.
    AbstractMap::setParams({mapSizeMeters,
                            cellSizeMeters,
                            minimumGapSizeMeters * minimumGapSizeMeters,
                            numCellsPerRowCol});

    VoronoiMap mapper;
    ros::Subscriber laserSub = nh.subscribe(localMapTopic, 1, &VoronoiMap::onNewData, &mapper);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}