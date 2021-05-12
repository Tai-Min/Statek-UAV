#include "../../include/gps_to_tf/fix_to_tf.hpp"
#include <tf/transform_broadcaster.h>

FixToTf::FixToTf(double originLat, double originLon, std::string _mapFrame, std::string _earthFrame)
    : originLambda(originLat * M_PI / 180.0),
      originPhi(originLon * M_PI / 180.0),
      mapFrame(_mapFrame),
      earthFrame(_earthFrame)
{
    geodeticToEcef(originLat, originLon, originEcefX, originEcefY, originEcefZ);
}

void FixToTf::geodeticToEcef(double lat, double lon, double &ecefX, double &ecefY, double &ecefZ)
{
    double lambda = lat * M_PI / 180.0;
    double phi = lon * M_PI / 180.0;
    double N = a / sqrt(1 - eSq * sin(lambda) * sin(lambda));

    ecefX = N * cos(lambda) * cos(phi);
    ecefY = N * cos(lambda) * sin(phi);
    ecefZ = ((b * b) / (a * a) * N) * sin(lambda);
}

void FixToTf::EcefToEnu(double ecefX, double ecefY, double ecefZ, double &enuX, double &enuY)
{
    double deltaEcefX = ecefX - this->originEcefX;
    double deltaEcefY = ecefY - this->originEcefY;
    double deltaEcefZ = ecefZ - this->originEcefZ;

    this->latestTangentX = -sin(originLambda) * deltaEcefX + cos(originLambda) * deltaEcefY;
    this->latestTangentY = -cos(originLambda) * sin(originPhi) * deltaEcefX - sin(originLambda) * sin(originPhi) * deltaEcefY + cos(originPhi) * deltaEcefZ;
}

void FixToTf::geodeticToEnu(double lat, double lon, double &enuX, double &enuY)
{
    double ecefX, ecefY, ecefZ;
    geodeticToEcef(lat, lon, ecefX, ecefY, ecefZ);
    EcefToEnu(ecefX, ecefY, ecefZ, enuX, enuY);
}

void FixToTf::onNewOdom(const nav_msgs::Odometry::ConstPtr &odom)
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

void FixToTf::onNewImu(const sensor_msgs::Imu::ConstPtr &imu)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu->orientation, quat);
    double temp1, temp2, theta;
    tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);
    this->latestYaw = theta - M_PI / 2.0;
    this->latestYaw = 0;
    std::cout << this->latestYaw << std::endl;
}

void FixToTf::onNewFix(const sensor_msgs::NavSatFix::ConstPtr &fix)
{
    if (isnan(fix->latitude) || isnan(fix->longitude))
        return;
    geodeticToEnu(fix->latitude, fix->longitude, this->latestTangentX, this->latestOdomTheta);

    this->latestTangentX = 2;
    this->latestTangentY = 1;
    std::cout << this->latestTangentX << ", " << this->latestTangentY << std::endl;

    // Fix updated so reset offset.
    this->odomOffsetX = 0;
    this->odomOffsetY = 0;
    this->odomOffsetTheta = 0;
}

geometry_msgs::TransformStamped FixToTf::getTransformMsg(const geometry_msgs::Transform &gpsToMap)
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