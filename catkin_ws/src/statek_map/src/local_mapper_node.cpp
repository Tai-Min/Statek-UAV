#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>

//#include <tf2_bullet/tf2_bullet.h>

#include "../include/laser_scan_map.hpp"
#include "../include/map_fuser.hpp"

geometry_msgs::TransformStamped getTransform(const std::string &frameId, const std::string &childFrameId, bool &ok)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener transformListener(tfBuffer);

    geometry_msgs::TransformStamped result;

    ok = true;
    try
    {
        result = tfBuffer.lookupTransform(frameId, childFrameId, ros::Time(0));
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
    std::string statekName;
    nh.param<std::string>("statek_name", statekName, "statek");

    std::string footprintFrame = statekName + "/base_footprint";

    std::string odomTopic = "/" + statekName + "/real_time/odom";
    std::string odomFrame = statekName + "/odom/odom_link";

    std::string laserTopic = "/" + statekName + "/laser/scan";
    std::string laserFrame = statekName + "/laser/laser_link";

    std::string mapTopic = "/" + statekName + "/map/local_map";
    std::string mapFrame = statekName + "/map/local_map";

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