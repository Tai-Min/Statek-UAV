#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "../../include/gps_to_tf/fix_to_tf.hpp"
#include "../../include/common.hpp"

int main(int argc, char **argv)
{
    // ROS init.
    ros::init(argc, argv, "gps_to_tf");
    ros::NodeHandle nh("~");

    // Get all the params.
    std::string statekName, gpsTopic, odomTopic, imuTopic, fixFilteredTopic, gpsFrame, mapFrame, earthFrame, geoToEnuService, enuToGeoService;
    double originLon, originLat, processVariance, measurementVariance;

    nh.param<std::string>("statek_name", statekName, "statek");

    nh.param<std::string>("gps_topic", gpsTopic, "/" + statekName + "/gps/fix");
    nh.param<std::string>("odom_topic", odomTopic, "/" + statekName + "/real_time/odom");
    nh.param<std::string>("imu_topic", imuTopic, "/" + statekName + "/real_time/imu");
    nh.param<std::string>("fix_filtered_topic", fixFilteredTopic, "/" + statekName + "/gps/fix_filtered");

    nh.param<std::string>("gps_frame", gpsFrame, statekName + "/gps/gps_link");
    nh.param<std::string>("map_frame", mapFrame, statekName + "/map/local_map_link");
    nh.param<std::string>("earth_frame", earthFrame, statekName + "/earth");

    nh.param<std::string>("geo_to_enu_service", geoToEnuService, statekName + "/geodetic_to_enu");
    nh.param<std::string>("enu_to_geo_service", enuToGeoService, statekName + "/enu_to_geodetic");

    nh.param<double>("origin_lon", originLon, 0.0);
    nh.param<double>("origin_lat", originLat, 0.0);

    nh.param<double>("process_variance", processVariance, 0.001);
    nh.param<double>("measurement_variance", measurementVariance, 15);

    // The converter.
    FixToTf converter(originLat, originLon, processVariance, measurementVariance, mapFrame, earthFrame);

    // Subscribers.
    ros::Subscriber gpsFixSub = nh.subscribe(gpsTopic, 1, &FixToTf::onNewFix, &converter);
    ros::Subscriber odomSub = nh.subscribe(odomTopic, 1, &FixToTf::onNewOdom, &converter);
    ros::Subscriber imuSub = nh.subscribe(imuTopic, 1, &FixToTf::onNewImu, &converter);

    // Publishers.
    ros::Publisher fixFilteredPub = nh.advertise<sensor_msgs::NavSatFix>(fixFilteredTopic, 1);

    // Services.
    ros::ServiceServer geoToEnuSrv = nh.advertiseService(geoToEnuService, &FixToTf::geodeticToEnuService, &converter);
    ros::ServiceServer enuToGeoSrv = nh.advertiseService(enuToGeoService, &FixToTf::enuToGeodeticService, &converter);

    // TF.
    tf2_ros::TransformBroadcaster transformBroadcaster;

    // The main loop.
    ros::Rate rate = ros::Rate(50);
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
        if(converter.newFixAvailable()){
            fixFilteredPub.publish(converter.getFilteredFixMsg());
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}