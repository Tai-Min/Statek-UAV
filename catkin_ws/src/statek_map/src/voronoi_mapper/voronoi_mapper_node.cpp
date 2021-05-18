#include <ros/ros.h>
#include "../../include/voronoi_mapper/voronoi_map.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <statek_map/Graph.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace
{
    double goalX, goalY;
}

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

void onNewShortTermGoal(const geometry_msgs::PoseStamped &goal)
{
    goalX = goal.pose.position.x;
    goalY = goal.pose.position.y;
}

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "local_mapper");
    ros::NodeHandle nh("~");

    // Get all the params.
    std::string statekName, shortTermGoalTopic, voronoiMapTopic, localMapTopic, earthFrame, localMapFrame, footprintFrame;
    double mapSizeMeters, cellSizeMeters, minimumGapSizeMeters;

    nh.param<std::string>("statek_name", statekName, "statek");

    nh.param<std::string>("short_term_goal_topic", shortTermGoalTopic, "/" + statekName + "/short_term_goal");
    nh.param<std::string>("voronoi_map_topic", voronoiMapTopic, "/" + statekName + "/map/voronoi_map");
    nh.param<std::string>("local_map_topic", localMapTopic, "/" + statekName + "/map/local_map");

    nh.param<std::string>("earth_frame", earthFrame, statekName + "/earth");
    nh.param<std::string>("local_map_frame", localMapFrame, statekName + "/map/local_map_link");
    nh.param<std::string>("footprint_frame", footprintFrame, statekName + "/base_footprint");

    nh.param<double>("map_size_meters", mapSizeMeters, 7);
    nh.param<double>("cell_size_meters", cellSizeMeters, 0.1);
    nh.param<double>("minimum_gap_size_meters", minimumGapSizeMeters, 1.1);

    // Set params.
    unsigned int numCellsPerRowCol = mapSizeMeters / cellSizeMeters;
    AbstractMap::setParams({mapSizeMeters,
                            cellSizeMeters,
                            minimumGapSizeMeters * minimumGapSizeMeters,
                            numCellsPerRowCol});

    VoronoiMap mapper;

    // Init subscribers and publishers.
    ros::Subscriber mapSub = nh.subscribe(localMapTopic, 1, &VoronoiMap::onNewLocalMap, &mapper);
    ros::Subscriber shortTermGoalSub = nh.subscribe(shortTermGoalTopic, 1, &onNewShortTermGoal);
    ros::Publisher voronoiPublisher = nh.advertise<statek_map::Graph>(voronoiMapTopic, 1);
    tf2_ros::TransformBroadcaster transformBroadcaster;

    // Main loop.
    ros::Rate rate = ros::Rate(20);
    while (ros::ok())
    {
        bool ok;
        geometry_msgs::TransformStamped t = getTransform(localMapFrame, earthFrame, ok);
        if (ok)
        {
            mapper.setTransform(t);
            mapper.setGoalPosition(goalX, goalY);
        }
        
        if(mapper.newGraphAvailable()){
            voronoiPublisher.publish(mapper.getGraph());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}