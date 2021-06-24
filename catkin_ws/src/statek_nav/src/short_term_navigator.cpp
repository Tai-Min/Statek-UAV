#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "../include/mpc.hpp"

void onNewPath(const nav_msgs::Path::ConstPtr &path)
{

}

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "local_mapper");
    ros::NodeHandle nh("~");

    // Get all the params.
    std::string statekName, pathTopic;
    double horizonDuration, horizonSampling;

    nh.param<std::string>("statek_name", statekName, "statek");
    nh.param<std::string>("path_topic", pathTopic, "/" + statekName + "short_term_path");

    nh.param<double>("horizon_duration", horizonDuration, 5);
    nh.param<double>("horizon_sampling", horizonSampling, 0.25);

    // Init subscribers and publishers.
    ros::Subscriber pathSub = nh.subscribe(pathTopic, 1, onNewPath);

    // The main loop.
    ros::Rate rate = ros::Rate(5);
    while (ros::ok())
    {
        


        ros::spinOnce();
        rate.sleep();
    }
}