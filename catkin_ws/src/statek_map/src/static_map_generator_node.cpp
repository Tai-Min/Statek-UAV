// TODO
/**
 * ODPALIC TO TAK ZEBY DZIALALO
 * SPRAWDZIC CZY TF = X * TRANS * ROT DZIALA JAK NALEZY NA PODSTAWIE NP LASER BASE LINK
 * JAK DZIALA TO KOMPENSOWAC W LASER MAP TRANSFORMACJE DO BASE FOOTPRINTA
 * ODCZYTYWAC OFFSET Z ODOMETRII I GO POTWIERDZIC
 * RAYCAST WOLNEJ PRZESTRZENI
 */
#include <ros/ros.h>

#include "../include/laser_scan_map.hpp"
#include "../include/map_fuser.hpp"

//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//ros::Publisher mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>(mapTopic, 1);
//ros::Publisher markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "static_map_generator");
    ros::NodeHandle nh("~");

    // Get all the params.
    std::string statekName = "statek";
    std::string footprintFrame = "/" + statekName + "/base_footprint";

    std::string laserTopic = "/" + statekName + "/laser/scan";
    std::string laserFrame = "/" + statekName + "/laser/laser_link";

    std::string mapTopic = "/" + statekName + "/map/local_static_map";
    std::string mapFrame = statekName + "/map/local_static_map";

    double mapSizeMeters = 7;
    double cellSizeMeters = 0.1;
    double minimumGapSizeMeters = 0.5;
    unsigned int numCellsPerRowCol = mapSizeMeters / cellSizeMeters;

    double mapUpdateRate = 1;

    // Save parameters so all mapping objects can access them.
    AbstractMap::setParams({mapSizeMeters,
                            cellSizeMeters,
                            minimumGapSizeMeters,
                            numCellsPerRowCol}); 

    // Create all mapping objects.
    LaserScanMap laserScanMap;
    MapFuser fuser(nh, mapTopic, mapFrame, mapUpdateRate, laserScanMap);

    // Init subscribers and publishers.
    nh.subscribe(laserTopic, 1, &LaserScanMap::onNewData, &laserScanMap);

    // The main loop.
    ros::Rate rate = ros::Rate(mapUpdateRate);
    while (ros::ok())
    {
        fuser.publish();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}