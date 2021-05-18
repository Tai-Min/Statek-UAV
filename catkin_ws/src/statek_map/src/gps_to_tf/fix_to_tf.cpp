#include "../../include/gps_to_tf/fix_to_tf.hpp"
#include <tf/transform_broadcaster.h>

FixToTf::FixToTf(double originLat, double originLon, double _northCompensation, std::string _mapFrame, std::string _earthFrame)
    : originLambda(originLat * M_PI / 180.0),
      originPhi(originLon * M_PI / 180.0),
      northCompensation(_northCompensation),
      mapFrame(_mapFrame),
      earthFrame(_earthFrame)
{
    geodeticToEcef(originLat, originLon, originEcefX, originEcefY, originEcefZ);
}

double FixToTf::sign(double v)
{
    return (v > 0) - (v < 0);
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

void FixToTf::EcefToGeodetic(double ecefX, double ecefY, double ecefZ, double &lat, double &lon)
{
    // ._.
    // Zhu's Algorithm.
    //https://hal.archives-ouvertes.fr/hal-01704943v2/document
    double w = sqrt(pow(ecefX, 2) + pow(ecefY, 2));
    double l = this->eSq / 2.0;
    double lSq = pow(l, 2);
    double m = pow((w / this->a), 2);
    double n = pow(((1 - this->eSq) * ecefZ / this->b), 2);
    double i = -(2 * lSq + m + n) / 2.0;
    double k = lSq * (lSq - m - n);
    double q = pow((m + n - 4 * lSq), 3) / 216.0 + m * n * lSq;
    double D = sqrt((2 * q - m * n * lSq) * m * n * lSq);
    double beta = i / 3.0 - cbrt(q + D) - cbrt(q - D);
    double t = sqrt(sqrt(pow(beta, 2) - k) - (beta + i) / 2.0) - this->sign(m - n) * sqrt((beta - i) / 2.0);
    double w1 = w / (t + l);
    double z1 = (1 - this->eSq) * ecefZ / (t - l);

    double lambda;
    if (w != 0)
        lambda = atan2(z1, (1 - this->eSq) * w1);
    else
        lambda = this->sign(z) * (M_PI / 2.0);

    double phi = 2 * atan2(w - ecefX, ecefY);

    lat = lambda * 180.0 / M_PI;
    lon = phi * 180.0 / M_PI;
}

void FixToTf::EcefToEnu(double ecefX, double ecefY, double ecefZ, double &enuX, double &enuY)
{
    double deltaEcefX = ecefX - this->originEcefX;
    double deltaEcefY = ecefY - this->originEcefY;
    double deltaEcefZ = ecefZ - this->originEcefZ;

    enuX = -sin(originLambda) * deltaEcefX + cos(originLambda) * deltaEcefY;
    enuY = -cos(originLambda) * sin(originPhi) * deltaEcefX - sin(originLambda) * sin(originPhi) * deltaEcefY + cos(originPhi) * deltaEcefZ;
}

void FixToTf::EnuToEcef(double enuX, double enuY, double &ecefX, double &ecefY, double &ecefZ)
{
    ecefX = (-sin(originPhi) * enuX - sin(originLambda) * cos(originPhi) * enuY) + this->originEcefX;
    ecefY = (cos(originPhi) * enuX - sin(originLambda) * sin(originPhi) * enuY) + this->originEcefY;
    ecefZ = (cos(originLambda) * enuY) + this->originEcefZ;
}

void FixToTf::geodeticToEnu(double lat, double lon, double &enuX, double &enuY)
{
    double ecefX, ecefY, ecefZ;
    this->geodeticToEcef(lat, lon, ecefX, ecefY, ecefZ);
    this->EcefToEnu(ecefX, ecefY, ecefZ, enuX, enuY);
}

void FixToTf::enuToGeodetic(double enuX, double enuY, double &lat, double &lon)
{
    double ecefX, ecefY, ecefZ;
    this->EnuToEcef(enuX, enuY, ecefX, ecefY, ecefZ);
    this->EcefToGeodetic(ecefX, ecefY, ecefZ, lat, lon);
}

void FixToTf::geodeticToEnuService()
{
}

void FixToTf::enuToGeodeticService()
{
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

    // Update odom offsets by travelled distance.
    this->odomOffsetX += travelledDistanceX;
    this->odomOffsetY += travelledDistanceY;

    // Save stuff.
    this->latestOdomX = odom->pose.pose.position.x;
    this->latestOdomY = odom->pose.pose.position.y;
}

void FixToTf::onNewImu(const sensor_msgs::Imu::ConstPtr &imu)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu->orientation, quat);
    double temp1, temp2, theta;
    tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);
    this->latestYaw = M_PI / 2.0 + theta - this->northCompensation;
    //std::cout << this->latestYaw * 180.0 / M_PI << std::endl;
}

void FixToTf::onNewFix(const sensor_msgs::NavSatFix::ConstPtr &fix)
{
    if (isnan(fix->latitude) || isnan(fix->longitude))
        return;

    std::cout << fix->latitude << ", " << fix->longitude << std::endl;
    geodeticToEnu(fix->latitude, fix->longitude, this->latestTangentX, this->latestTangentY);

    double lat, lon;
    enuToGeodetic(this->latestTangentX, this->latestTangentY, lat, lon);
    std::cout << lat << ", " << lon << std::endl;
    std::cout << "----------" << std::endl;

    // Fix updated so reset offset.
    this->odomOffsetX = 0;
    this->odomOffsetY = 0;
}

geometry_msgs::TransformStamped FixToTf::getTransformMsg(const geometry_msgs::Transform &gpsToMap)
{
    // Get transform from tangent plane to gps link.
    geometry_msgs::Transform tfTransformTangentMsg;
    tfTransformTangentMsg.translation.x = this->latestTangentX;
    tfTransformTangentMsg.translation.y = this->latestTangentY;
    tfTransformTangentMsg.rotation.x = 0;
    tfTransformTangentMsg.rotation.y = 0;
    tfTransformTangentMsg.rotation.z = 0;
    tfTransformTangentMsg.rotation.w = 1.0; // GPS does not provide rotation data.
    tf::Transform tfTransformTangent;
    tf::transformMsgToTF(tfTransformTangentMsg, tfTransformTangent);
    tfTransformTangent.inverse();

    // Accommodate offset of odometry.
    geometry_msgs::Transform tfTransformOdomMsg;
    tfTransformOdomMsg.translation.x = this->odomOffsetX;
    tfTransformOdomMsg.translation.y = this->odomOffsetY;
    tf2::Quaternion q;
    q.setRPY(0, 0, this->latestYaw); // For rotation use IMU as it is facing true north and updating fast enough.
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