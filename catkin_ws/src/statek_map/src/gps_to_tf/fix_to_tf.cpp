#include "../../include/gps_to_tf/fix_to_tf.hpp"
#include <tf/transform_broadcaster.h>

FixToTf::FixToTf(double originLat, double originLon,
                 double processVarianceNorth, double processVarianceEast,
                 double measurementVarianceNorth, double measurementVarianceEast,
                 std::string _mapFrame, std::string _earthFrame)
    : originPhi(originLat * M_PI / (double)180.0),
      originLambda(originLon * M_PI / (double)180.0),
      mapFrame(_mapFrame),
      earthFrame(_earthFrame),
      filter(processVarianceNorth, processVarianceEast, measurementVarianceNorth, measurementVarianceEast)
{
    geodeticToEcef(originLat, originLon, originEcefX, originEcefY, originEcefZ);
    shortTermGoalCartesian.header.frame_id = earthFrame;
}

double FixToTf::sign(double v)
{
    return (v > 0) - (v < 0);
}

void FixToTf::geodeticToEcef(double lat, double lon, double &ecefX, double &ecefY, double &ecefZ)
{
    double phi = lat * M_PI / (double)180.0;
    double lambda = lon * M_PI / (double)180.0;
    double N = a / sqrt((double)1.0 - eSq * pow(sin(phi), 2));

    ecefX = N * cos(phi) * cos(lambda);
    ecefY = N * cos(phi) * sin(lambda);
    ecefZ = (pow(b, 2)) / (pow(a, 2)) * N * sin(phi);
}

void FixToTf::EcefToGeodetic(double ecefX, double ecefY, double ecefZ, double &lat, double &lon)
{
    // ._.
    // Zhu's Algorithm.
    // https://hal.archives-ouvertes.fr/hal-01704943v2/document
    double w = sqrt(pow(ecefX, 2) + pow(ecefY, 2));
    double l = eSq / (double)2.0;
    double lSq = pow(l, 2);
    double m = pow(w / a, 2);
    double n = pow((((double)1.0 - eSq) * ecefZ / b), 2);
    double i = -((double)2 * lSq + m + n) / (double)2.0;
    double k = lSq * (lSq - m - n);
    double q = pow((m + n - (double)4.0 * lSq), 3) / (double)216.0 + m * n * lSq;
    double D = sqrt(fabs(((double)2.0 * q - m * n * lSq) * m * n * lSq));
    double beta = i / (double)3.0 - cbrt(q + D) - cbrt(q - D);
    double t = sqrt(sqrt(pow(beta, 2) - k) - (beta + i) / (double)2.0) - sign(m - n) * sqrt(fabs((beta - i) / (double)2.0));
    double w1 = w / (t + l);
    double z1 = ((double)1 - eSq) * ecefZ / (t - l);

    double phi;
    if (w != 0)
        phi = atan2(z1, ((double)1.0 - eSq) * w1);
    else
        phi = sign(ecefZ) * (M_PI / (double)2.0);
    double lambda = (double)2.0 * atan2(w - ecefX, ecefY);

    lat = phi * (double)180.0 / M_PI;
    lon = lambda * (double)180.0 / M_PI;
}

void FixToTf::EcefToEnu(double ecefX, double ecefY, double ecefZ, double &enuX, double &enuY, double &enuZ) const
{
    double deltaEcefX = ecefX - this->originEcefX;
    double deltaEcefY = ecefY - this->originEcefY;
    double deltaEcefZ = ecefZ - this->originEcefZ;

    enuX = -sin(originLambda) * deltaEcefX + cos(originLambda) * deltaEcefY;
    enuY = -sin(originPhi) * cos(originLambda) * deltaEcefX - sin(originPhi) * sin(originLambda) * deltaEcefY + cos(originPhi) * deltaEcefZ;
    enuZ = cos(originPhi) * cos(originLambda) * deltaEcefX + cos(originPhi) * sin(originLambda) * deltaEcefY + sin(originPhi) * deltaEcefZ;
}

void FixToTf::EnuToEcef(double enuX, double enuY, double enuZ, double &ecefX, double &ecefY, double &ecefZ) const
{
    ecefX = (-sin(originLambda) * enuX - sin(originPhi) * cos(originLambda) * enuY + cos(originPhi) * cos(originLambda) * enuZ) + this->originEcefX;
    ecefY = (cos(originLambda) * enuX - sin(originPhi) * sin(originLambda) * enuY + cos(originPhi) * sin(originLambda) * enuZ) + this->originEcefY;
    ecefZ = (cos(originPhi) * enuY + sin(originPhi) * enuZ) + this->originEcefZ;
}

void FixToTf::geodeticToEnu(double lat, double lon, double &enuX, double &enuY, double &enuZ) const
{
    double ecefX, ecefY, ecefZ;
    this->geodeticToEcef(lat, lon, ecefX, ecefY, ecefZ);
    this->EcefToEnu(ecefX, ecefY, ecefZ, enuX, enuY, enuZ);
}

void FixToTf::enuToGeodetic(double enuX, double enuY, double enuZ, double &lat, double &lon) const
{
    double ecefX, ecefY, ecefZ;
    this->EnuToEcef(enuX, enuY, enuZ, ecefX, ecefY, ecefZ);
    this->EcefToGeodetic(ecefX, ecefY, ecefZ, lat, lon);
}

bool FixToTf::newFixAvailable() const
{
    return isNewFixAvailable;
}

bool FixToTf::newGoalCartesianAvailable() const
{
    return isNewGoalCartesianAvailable;
}

bool FixToTf::geodeticToEnuService(statek_map::GeoToEnu::Request &req,
                                   statek_map::GeoToEnu::Response &res)
{
    double tempX, tempY, tempZ;
    geodeticToEnu(req.latitude, req.longitude, tempX, tempY, tempZ);
    res.x = tempX;
    res.y = tempY;
    res.z = tempZ;
}

bool FixToTf::enuToGeodeticService(statek_map::EnuToGeo::Request &req,
                                   statek_map::EnuToGeo::Response &res)
{
    double tempLat, tempLon;
    enuToGeodetic(req.x, req.y, req.z, tempLat, tempLon);
    res.latitude = tempLat;
    res.longitude = tempLon;
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
    double rotation = theta - this->latestOdomTheta;

    // Update odom offsets by travelled distance.
    this->odomOffsetX += travelledDistanceX;
    this->odomOffsetY += travelledDistanceY;

    // Save stuff.
    this->latestOdomX = odom->pose.pose.position.x;
    this->latestOdomY = odom->pose.pose.position.y;
    this->latestOdomTheta = theta;
}

void FixToTf::onNewImu(const sensor_msgs::Imu::ConstPtr &imu)
{
    // Convert acceleration of the imu (the one facing forward in robot frame)
    // to north / east directions.
    // In this robot the -Y points forward.
    double linAcc = -imu->linear_acceleration.y;
    this->latestAccelerationNorth = linAcc * sin(this->latestYaw);
    this->latestAccelerationEast = linAcc * cos(this->latestYaw);

    // Save angular difference from true north.
    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu->orientation, quat);
    double temp1, temp2, theta;
    tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);
    this->latestYaw = theta;
}

void FixToTf::onNewFix(const sensor_msgs::NavSatFix::ConstPtr &fix)
{
    if (isnan(fix->latitude) || isnan(fix->longitude))
        return;

    geodeticToEnu(fix->latitude, fix->longitude, this->latestTangentX, this->latestTangentY, this->latestTangentZ);

    Kalman::Estimates estimates = this->filter.update({this->latestAccelerationEast, this->latestAccelerationNorth},
                                                      {this->latestTangentX, this->latestTangentY});
    this->latestTangentX = estimates.x;
    this->latestTangentY = estimates.y;

    // Convert filtered tangents back into fix.
    double tempLat, tempLon;
    enuToGeodetic(this->latestTangentX, this->latestTangentY, this->latestTangentZ, tempLat, tempLon);

    fixFiltered.latitude = tempLat;
    fixFiltered.longitude = tempLon;

    fixFiltered.header = fix->header;
    fixFiltered.status = fix->status;
    isNewFixAvailable = true;

    // Fix updated so reset offsets.
    this->odomOffsetX = 0;
    this->odomOffsetY = 0;
}

void FixToTf::onNewPoseGps(const sensor_msgs::NavSatFix::ConstPtr &fix)
{
    if (isnan(fix->latitude) || isnan(fix->longitude))
        return;

    double tangentX, tangentY, tangentZ;
    geodeticToEnu(fix->latitude, fix->longitude, tangentX, tangentY, tangentZ);

    shortTermGoalCartesian.pose.position.x = tangentX;
    shortTermGoalCartesian.pose.position.y = tangentY;

    isNewGoalCartesianAvailable = true;
}

geometry_msgs::TransformStamped FixToTf::getTransformMsg(const geometry_msgs::Transform &gpsToMap) const
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

    // Accommodate offset of odometry.
    geometry_msgs::Transform tfTransformOdomMsg;
    tfTransformOdomMsg.translation.x = this->odomOffsetX;
    tfTransformOdomMsg.translation.y = this->odomOffsetY;
    tf2::Quaternion q;
    q.setRPY(0, 0, this->latestYaw); // For rotation use IMU as it is facing true north and updating fast enough.
    q.normalize();
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

const sensor_msgs::NavSatFix &FixToTf::getFilteredFixMsg()
{
    isNewFixAvailable = false;
    fixFiltered.header.stamp = ros::Time::now();
    return fixFiltered;
}

const geometry_msgs::PoseStamped &FixToTf::getPoseCartesian()
{
    isNewGoalCartesianAvailable = false;
    shortTermGoalCartesian.header.stamp = ros::Time::now();
    return shortTermGoalCartesian;
}