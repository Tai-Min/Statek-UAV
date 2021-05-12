#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "../../include/local_mapper/laser_scan_map.hpp"
#include "../../include/local_mapper/map_fuser.hpp"

/**
 * @brief Get some transform.
 * @param targetFrame Target of transform.
 * @param sourceFrame Source of transform.
 * @param ok Set to true on success, false otherwise.
 * @return If ok then requested transform, otherwise unspecified.
 */
geometry_msgs::TransformStamped getTransform(const std::string &targetFrame, const std::string &sourceFrame, bool &ok)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener transformListener(tfBuffer);

    geometry_msgs::TransformStamped result;

    ok = true;
    try
    {
        result = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    }
    catch (tf::TransformException ex)
    {
        ok = false;
    }

    return result;
}

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "local_mapper");
    ros::NodeHandle nh("~");

    // Get all the params.
    std::string statekName, footprintFrame, odomTopic, odomFrame, laserTopic, laserFrame, mapTopic, mapFrame;
    double mapSizeMeters, cellSizeMeters, minimumGapSizeMeters;
    int mapUpdateRateMs;

    nh.param<std::string>("statek_name", statekName, "statek");

    nh.param<std::string>("footprint_frame", footprintFrame, statekName + "/base_footprint");
    nh.param<std::string>("odom_topic", odomTopic, "/" + statekName + "/real_time/odom");
    nh.param<std::string>("odom_frame", odomFrame, statekName + "/odom/odom_link");
    nh.param<std::string>("laser_topic", laserTopic, "/" + statekName + "/laser/scan");
    nh.param<std::string>("laser_frame", laserFrame, statekName + "/laser/laser_link");
    nh.param<std::string>("local_map_topic", mapTopic, "/" + statekName + "/map/local_map");
    nh.param<std::string>("local_map_frame", mapFrame, statekName + "/map/local_map_link");

    nh.param<double>("map_size_meters", mapSizeMeters, 7);
    nh.param<double>("cell_size_meters", cellSizeMeters, 0.1);
    nh.param<double>("minimum_gap_size_meters", minimumGapSizeMeters, 1.1);

    nh.param<int>("map_update_rate_ms", mapUpdateRateMs, 20);

    // Save parameters so all mapping objects can access them.
    unsigned int numCellsPerRowCol = mapSizeMeters / cellSizeMeters;
    AbstractMap::setParams({mapSizeMeters,
                            cellSizeMeters,
                            minimumGapSizeMeters * minimumGapSizeMeters,
                            numCellsPerRowCol});

    // Create all mapping objects.
    LaserScanMap laserScanMap;

    // Map fuser.
    MapFuser fuser(odomFrame, mapFrame, mapUpdateRateMs, {laserScanMap});

    // Init subscribers and publishers.
    ros::Subscriber laserSub = nh.subscribe(laserTopic, 1, &LaserScanMap::onNewData, &laserScanMap);
    ros::Subscriber odomSub = nh.subscribe(odomTopic, 1, &MapFuser::onNewOdom, &fuser);
    ros::Publisher mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>(mapTopic, 1);
    tf2_ros::TransformBroadcaster transformBroadcaster;

    // The main loop.
    ros::Rate rate = ros::Rate(20);
    while (ros::ok())
    {
        // Update transforms
        bool ok;
        geometry_msgs::TransformStamped t = getTransform(footprintFrame, laserFrame, ok);
        if (ok)
            laserScanMap.setTransform(t);

        // Update map.
        if (fuser.tryUpdateMap())
        {
            mapPublisher.publish(fuser.getMapMsg());
        }

        // Send new transforms.
        transformBroadcaster.sendTransform(fuser.getTransformMsg());

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}