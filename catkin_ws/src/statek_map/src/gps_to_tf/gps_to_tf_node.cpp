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
    std::string statekName, gpsTopic, odomTopic, imuTopic, gpsFrame, mapFrame, earthFrame, geoToEnuService, enuToGeoService;
    double originLonDeg, originLonMin, originLonSec, originLatDeg, originLatMin, originLatSec, northCompensation;

    nh.param<std::string>("statek_name", statekName, "statek");

    nh.param<std::string>("gps_topic", gpsTopic, "/" + statekName + "/gps/fix");
    nh.param<std::string>("odom_topic", odomTopic, "/" + statekName + "/real_time/odom");
    nh.param<std::string>("imu_topic", imuTopic, "/" + statekName + "/real_time/imu");

    nh.param<std::string>("gps_frame", gpsFrame, statekName + "/gps/gps_link");
    nh.param<std::string>("map_frame", mapFrame, statekName + "/map/local_map_link");
    nh.param<std::string>("earth_frame", earthFrame, statekName + "/earth");

    nh.param<std::string>("geo_to_enu_service", geoToEnuService, statekName + "/geodetic_to_enu");
    nh.param<std::string>("enu_to_geo_service", enuToGeoService, statekName + "/enu_to_geodetic");

    nh.param<double>("origin_lon_deg", originLonDeg, 0.0);
    nh.param<double>("origin_lon_min", originLonMin, 0.0);
    nh.param<double>("origin_lon_sec", originLonSec, 0.0);

    nh.param<double>("origin_lat_deg", originLatDeg, 0.0);
    nh.param<double>("origin_lat_min", originLatMin, 0.0);
    nh.param<double>("origin_lat_sec", originLatSec, 0.0);

    nh.param<double>("north_compensation", northCompensation, 2.3561944902);

    // The converter.
    double originLat = originLatDeg + originLatMin / 60.0 + originLatSec / 3600.0;
    double originLon = originLonDeg + originLonMin / 60.0 + originLonSec / 3600.0;
    FixToTf converter(originLat, originLon, northCompensation, mapFrame, earthFrame);

    // Subscribers.
    ros::Subscriber gpsFixSub = nh.subscribe(gpsTopic, 1, &FixToTf::onNewFix, &converter);
    ros::Subscriber odomSub = nh.subscribe(odomTopic, 1, &FixToTf::onNewOdom, &converter);
    ros::Subscriber imuSub = nh.subscribe(imuTopic, 1, &FixToTf::onNewImu, &converter);

    // Services.
    ros::ServiceServer geoToEnuSrv = nh.advertiseService(geoToEnuService, &FixToTf::geodeticToEnuService, &converter);
    ros::ServiceServer enuToGeoSrv = nh.advertiseService(enuToGeoService, &FixToTf::enuToGeodeticService, &converter);

    // TF.
    tf2_ros::TransformBroadcaster transformBroadcaster;

    // The main loop.
    ros::Rate rate = ros::Rate(20);
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
        rate.sleep();
    }
    return 0;
}