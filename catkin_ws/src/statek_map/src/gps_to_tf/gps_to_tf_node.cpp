#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "../../include/gps_to_tf/fix_to_tf.hpp"

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
    ros::init(argc, argv, "gps_to_tf");
    ros::NodeHandle nh("~");

    // Get all the params.
    std::string statekName;
    nh.param<std::string>("statek_name", statekName, "statek");

    double originLonDeg, originLonMin, originLonSec, originLon;
    nh.param<double>("origin_lon_deg", originLonDeg, 0.0);
    nh.param<double>("origin_lon_min", originLonMin, 0.0);
    nh.param<double>("origin_lon_sec", originLonSec, 0.0);
    originLon = originLonDeg + originLonMin / 60.0 + originLonSec / 3600.0;

    double originLatDeg, originLatMin, originLatSec, originLat;
    nh.param<double>("origin_lat_deg", originLatDeg, 0.0);
    nh.param<double>("origin_lat_min", originLatMin, 0.0);
    nh.param<double>("origin_lat_sec", originLatSec, 0.0);
    originLat = originLatDeg + originLatMin / 60.0 + originLatSec / 3600.0;

    std::string gpsTopic = "/" + statekName + "/gps/fix";
    std::string odomTopic = "/" + statekName + "/real_time/odom";
    std::string imuTopic = "/" + statekName + "/real_time/imu";

    std::string gpsFrame = statekName + "/gps/gps_link";
    std::string mapFrame = statekName + "/map/local_map_link";
    std::string earthFrame = statekName + "/earth";

    // The converter.
    FixToTf converter(originLat, originLon, mapFrame, earthFrame);

    // Subscribers.
    ros::Subscriber gpsFixSub = nh.subscribe(gpsTopic, 1, &FixToTf::onNewFix, &converter);
    ros::Subscriber odomSub = nh.subscribe(odomTopic, 1, &FixToTf::onNewOdom, &converter);
    ros::Subscriber imuSub = nh.subscribe(imuTopic, 1, &FixToTf::onNewImu, &converter);

    tf2_ros::TransformBroadcaster transformBroadcaster;

    // The main loop.
    while (ros::ok())
    {
        // Update transforms.
        bool ok;
        geometry_msgs::TransformStamped t = getTransform(gpsFrame, mapFrame, ok);
        if (ok)
        {
            // Send new transforms.
            transformBroadcaster.sendTransform(converter.getTransformMsg(t.transform));
        }

        ros::spinOnce();
    }
    return 0;
}