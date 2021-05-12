#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <iostream>

/**
 * @brief Convert from GPS coordinates to some cartesian local tangent plane.
 */
class FixToTf
{
private:
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
    // https://archive.psas.pdx.edu/CoordinateSystem/Latitude_to_LocalTangent.pdf
    // Some params.
    static constexpr double a = 6378137.0;               //!< WGS-84 semi major axis in meters.
    static constexpr double b = 6356752.314245;          //!< Semi minor axis in meters.
    static constexpr double eSq = 1 - (b * b) / (a * a); //!< Square of eccentricity.

    std::string mapFrame;
    std::string earthFrame;

    // Origin of tangent plane in lon / lat converted to radians and ECEF
    double originPhi = 0;    //!< Origin longitude in radians.
    double originLambda = 0; //!< Origin latitude in radians.
    double originEcefX = 0;
    double originEcefY = 0;
    double originEcefZ = 0;
    double latestTangentX = 0;
    double latestTangentY = 0;
    double latestYaw = 0;

    // Odometry memory for compensation.
    double latestOdomX = 0;     //!< Latest odometry reading. Required to compute travelled distance since previous odom.
    double latestOdomY = 0;     //!< Latest odometry reading. Required to compute travelled distance since previous odom.
    double latestOdomTheta = 0; //!< Latest odometry reading. Required to compute travelled distance since previous odom.

    // Odometry compensation.
    double odomOffsetX = 0;     //!< Accomodate movement in x direction between GPS updates.
    double odomOffsetY = 0;     //!< Accomodate movement in y direction between GPS updates.
    double odomOffsetTheta = 0; //!< Accomodate rotation between GPS updates.

    /**
     * @brief Convert latitude and longitude to ECEF x, y, z coordinates.
     * @param lat Latitude.
     * @param lon longitude.
     * @param ecefX ECEF x.
     * @param ecefY ECEF y.
     * @param ecefZ ECEF z.
     */
    static void geodeticToEcef(double lat, double lon, double &ecefX, double &ecefY, double &ecefZ)
    {
        double lambda = lat * M_PI / 180.0;
        double phi = lon * M_PI / 180.0;
        double N = a / sqrt(1 - eSq * sin(lambda) * sin(lambda));

        ecefX = N * cos(lambda) * cos(phi);
        ecefY = N * cos(lambda) * sin(phi);
        ecefZ = ((b * b) / (a * a) * N) * sin(lambda);
    }

    /**
     * @brief Convert ECEF coordinates to ENU.
     * @param ecefX ECEF x.
     * @param ecefY ECEF y.
     * @param ecefZ ECEF z
     * @param enuX ENU x.
     * @param enuY Enu y.
     */
    void EcefToEnu(double ecefX, double ecefY, double ecefZ)
    {
        double deltaEcefX = ecefX - this->originEcefX;
        double deltaEcefY = ecefY - this->originEcefY;
        double deltaEcefZ = ecefZ - this->originEcefZ;

        this->latestTangentX = -sin(originLambda) * deltaEcefX + cos(originLambda) * deltaEcefY;
        this->latestTangentY = -cos(originLambda) * sin(originPhi) * deltaEcefX - sin(originLambda) * sin(originPhi) * deltaEcefY + cos(originPhi) * deltaEcefZ;
    }

    /**
     * @brief Convert GPS coordinates to ENU.
     * @param lat Latitude.
     * @param lon longitude.
     * @param enuX ENU x.
     * @param enuY ENU y.
     */
    void geodeticToEnu(double lat, double lon)
    {
        double ecefX, ecefY, ecefZ;
        geodeticToEcef(lat, lon, ecefX, ecefY, ecefZ);
        EcefToEnu(ecefX, ecefY, ecefZ);
    }

public:
    /**
     * @brief Class constructor.
     * @param originLat Latitude for the center of tangent plane.
     * @param originLon Longtitute of the center of tangent plane.
     */
    FixToTf(double originLat, double originLon, std::string _mapFrame, std::string _earthFrame) : originLambda(originLat * M_PI / 180.0), originPhi(originLon * M_PI / 180.0), mapFrame(_mapFrame), earthFrame(_earthFrame)
    {
        geodeticToEcef(originLat, originLon, originEcefX, originEcefY, originEcefZ);
    }

    /**
     * @brief Callback for odom subscriber.
     * @param odom Current odometry.
     */
    void onNewOdom(const nav_msgs::Odometry::ConstPtr &odom)
    {
        // Convert to tf quaternion.
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);

        // Convert to RPY angles.
        double temp1, temp2, theta;
        tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);

        // Get travelled distance since last odom msg.
        double travelledDistanceX = odom->pose.pose.position.x - this->latestOdomX;
        double travelledDistanceY = odom->pose.pose.position.y - this->latestOdomY;
        double travelledDistanceTheta = theta - this->latestOdomTheta;

        // Update odom offsets by travelled distance.
        this->odomOffsetX += travelledDistanceX;
        this->odomOffsetY += travelledDistanceY;
        this->odomOffsetTheta += travelledDistanceTheta;

        // Save stuff.
        this->latestOdomX = odom->pose.pose.position.x;
        this->latestOdomY = odom->pose.pose.position.y;
        this->latestOdomTheta = theta;
    }

    void onNewImu(const sensor_msgs::Imu::ConstPtr &imu){
        tf::Quaternion quat;
        tf::quaternionMsgToTF(imu->orientation, quat);
        double temp1, temp2, theta;
        tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);
        this->latestYaw = theta - M_PI / 2.0;
        this->latestYaw = 0;
        std::cout << this->latestYaw << std::endl;
    }

    /**
     * @brief Called on new GPS fix.
     * @param fix Fix.
     */
    void onNewFix(const sensor_msgs::NavSatFix::ConstPtr &fix)
    {
        if (isnan(fix->latitude) || isnan(fix->longitude))
            return;
        geodeticToEnu(fix->latitude, fix->longitude);

        this->latestTangentX = 2;
        this->latestTangentY = 1;
        std::cout << this->latestTangentX << ", " << this->latestTangentY << std::endl;

        // Fix updated so reset offset.
        this->odomOffsetX = 0;
        this->odomOffsetY = 0;
        this->odomOffsetTheta = 0;
    }

    geometry_msgs::TransformStamped getTransformMsg(const geometry_msgs::Transform &gpsToMap)
    {
        // Get transform from tangent plane to gps link.
        geometry_msgs::Transform tfTransformTangentMsg;
        tfTransformTangentMsg.translation.x = this->latestTangentX;
        tfTransformTangentMsg.translation.y = this->latestTangentY;
        tf2::Quaternion q;
        q.setRPY(0, 0, this->latestYaw);
        tfTransformTangentMsg.rotation.x = q.x();
        tfTransformTangentMsg.rotation.y = q.y();
        tfTransformTangentMsg.rotation.z = q.z();
        tfTransformTangentMsg.rotation.w = q.w();
        tf::Transform tfTransformTangent;
        tf::transformMsgToTF(tfTransformTangentMsg, tfTransformTangent);
        tfTransformTangent.inverse();

        // Accommodate offset of odometry.
        geometry_msgs::Transform tfTransformOdomMsg;
        tfTransformOdomMsg.translation.x = this->odomOffsetX;
        tfTransformOdomMsg.translation.y = this->odomOffsetY;
        q.setRPY(0, 0, this->odomOffsetTheta);
        tfTransformOdomMsg.rotation.x = q.x();
        tfTransformOdomMsg.rotation.y = q.y();
        tfTransformOdomMsg.rotation.z = q.z();
        tfTransformOdomMsg.rotation.w = q.w();
        tf::Transform tfTransformOdom;
        tf::transformMsgToTF(tfTransformOdomMsg, tfTransformOdom);
        tfTransformOdom.inverse();

        // Invert transform from gps link to map link to compensate it in resulting transform.
        tf::Transform tfTransformGps;
        tf::transformMsgToTF(gpsToMap, tfTransformGps);
        tfTransformGps.inverse();

        // Get final transform.
        geometry_msgs::TransformStamped result;
        result.header.frame_id = earthFrame;
        result.child_frame_id = mapFrame;
        result.header.stamp = ros::Time::now();
        tf::transformTFToMsg(tfTransformTangent * tfTransformOdom * tfTransformGps, result.transform);

        // GPS to footprint probably has some height difference.
        result.transform.translation.z = 0;
        return result;
    }
};

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
    //std::string gpsTopic = "/fix";
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

    // The main loop
    while (ros::ok())
    {
        // Update transforms
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