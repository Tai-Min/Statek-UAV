#include "../../include/gps_to_tf/fix_to_tf.hpp"
#include <tf/transform_broadcaster.h>

FixToTf::FixToTf(long double originLat, long double originLon, double _northCompensation,
                 double processVariance, double measurementVariance,
                 std::string _mapFrame, std::string _earthFrame)
    : originPhi(originLat * M_PIf128 / (long double)180.0),
      originLambda(originLon * M_PIf128 / (long double)180.0),
      northCompensation(_northCompensation),
      mapFrame(_mapFrame),
      earthFrame(_earthFrame),
      filter(processVariance, measurementVariance)
{
    geodeticToEcef(originLat, originLon, originEcefX, originEcefY, originEcefZ);
}

long double FixToTf::sign(long double v)
{
    return (v > 0) - (v < 0);
}

void FixToTf::geodeticToEcef(long double lat, long double lon, long double &ecefX, long double &ecefY, long double &ecefZ)
{
    long double phi = lat * M_PIf128 / (long double)180.0;
    long double lambda = lon * M_PIf128 / (long double)180.0;
    long double N = a / sqrtf128((long double)1.0 - eSq * powf128(sinf128(phi), 2));

    ecefX = N * cosf128(phi) * cosf128(lambda);
    ecefY = N * cosf128(phi) * sinf128(lambda);
    ecefZ = (powf128(b, 2)) / (powf128(a, 2)) * N * sinf128(phi);
}

void FixToTf::EcefToGeodetic(long double ecefX, long double ecefY, long double ecefZ, long double &lat, long double &lon)
{
    // ._.
    // Zhu's Algorithm.
    // https://hal.archives-ouvertes.fr/hal-01704943v2/document
    long double w = sqrtf128(powf128(ecefX, 2) + powf128(ecefY, 2));
    long double l = eSq / (long double)2.0;
    long double lSq = powf128(l, 2);
    long double m = powf128(w / a, 2);
    long double n = powf128((((long double)1.0 - eSq) * ecefZ / b), 2);
    long double i = -((long double)2 * lSq + m + n) / (long double)2.0;
    long double k = lSq * (lSq - m - n);
    long double q = powf128((m + n - (long double)4.0 * lSq), 3) / (long double)216.0 + m * n * lSq;
    long double D = sqrtf128(fabsf128(((long double)2.0 * q - m * n * lSq) * m * n * lSq));
    long double beta = i / (long double)3.0 - cbrtf128(q + D) - cbrtf128(q - D);
    long double t = sqrtf128(sqrtf128(powf128(beta, 2) - k) - (beta + i) / (long double)2.0) - sign(m - n) * sqrtf128(fabsf128((beta - i) / (long double)2.0));
    long double w1 = w / (t + l);
    long double z1 = ((long double)1 - eSq) * ecefZ / (t - l);

    long double phi;
    if (w != 0)
        phi = atan2f128(z1, ((long double)1.0 - eSq) * w1);
    else
        phi = sign(ecefZ) * (M_PIf128 / (long double)2.0);
    long double lambda = (long double)2.0 * atan2f128(w - ecefX, ecefY);

    lat = phi * (long double)180.0 / M_PIf128;
    lon = lambda * (long double)180.0 / M_PIf128;
}

void FixToTf::EcefToEnu(long double ecefX, long double ecefY, long double ecefZ, long double &enuX, long double &enuY, long double &enuZ) const
{
    long double deltaEcefX = ecefX - this->originEcefX;
    long double deltaEcefY = ecefY - this->originEcefY;
    long double deltaEcefZ = ecefZ - this->originEcefZ;

    enuX = -sinf128(originPhi) * deltaEcefX + cosf128(originPhi) * deltaEcefY;
    enuY = -cosf128(originPhi) * sinf128(originLambda) * deltaEcefX - sinf128(originPhi) * sinf128(originLambda) * deltaEcefY + cosf128(originLambda) * deltaEcefZ;
    enuZ = cosf128(originPhi) * cosf128(originLambda) * deltaEcefX + cosf128(originPhi) * sinf128(originLambda) * deltaEcefY + sinf128(originPhi) * deltaEcefZ;
}

void FixToTf::EnuToEcef(long double enuX, long double enuY, long double enuZ, long double &ecefX, long double &ecefY, long double &ecefZ) const
{
    ecefX = (-sinf128(originLambda) * enuX - sinf128(originPhi) * cosf128(originLambda) * enuY + cosf128(originPhi) * cosf128(originLambda) * enuZ) + this->originEcefX;
    ecefY = (cosf128(originLambda) * enuX - sinf128(originPhi) * sinf128(originLambda) * enuY + cosf128(originPhi) * sinf128(originLambda) * enuZ) + this->originEcefY;
    ecefZ = (cosf128(originPhi) * enuY + sinf128(originPhi) * enuZ) + this->originEcefZ;
}

void FixToTf::geodeticToEnu(long double lat, long double lon, long double &enuX, long double &enuY, long double &enuZ) const
{
    long double ecefX, ecefY, ecefZ;
    this->geodeticToEcef(lat, lon, ecefX, ecefY, ecefZ);
    this->EcefToEnu(ecefX, ecefY, ecefZ, enuX, enuY, enuZ);
}

void FixToTf::enuToGeodetic(long double enuX, long double enuY, long double enuZ, long double &lat, long double &lon) const
{
    long double ecefX, ecefY, ecefZ;
    this->EnuToEcef(enuX, enuY, enuZ, ecefX, ecefY, ecefZ);
    this->EcefToGeodetic(ecefX, ecefY, ecefZ, lat, lon);
}

bool FixToTf::newFixAvailable() const
{
    return isNewFixAvailable;
}

bool FixToTf::geodeticToEnuService(statek_map::GeoToEnu::Request &req,
                                   statek_map::GeoToEnu::Response &res)
{
    long double tempX, tempY, tempZ;
    geodeticToEnu(req.latitude, req.longitude, tempX, tempY, tempZ);
    res.x = tempX;
    res.y = tempY;
    res.z = tempZ;
}

bool FixToTf::enuToGeodeticService(statek_map::EnuToGeo::Request &req,
                                   statek_map::EnuToGeo::Response &res)
{
    long double tempLat, tempLon;
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
    this->odomOffsetTheta += rotation;

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
    this->latestYaw = 1.5 * M_PI + theta - this->northCompensation;

    // Use filter only after first fix to get good initial values.
    if (!fstFix)
    {
        Kalman::Estimates estimates = this->filter.update({(double)this->latestAccelerationEast, (double)this->latestAccelerationNorth, (double)this->odomOffsetTheta},
                                                          {(double)this->latestTangentX, (double)this->latestTangentY, (double)this->latestYaw});
        //this->latestYaw = estimates.yaw;

        // IMU updated so reset offset.
        this->odomOffsetTheta = 0;
    }
}

void FixToTf::onNewFix(const sensor_msgs::NavSatFix::ConstPtr &fix)
{
    this->fstFix = false;

    fixFiltered.header = fix->header;
    fixFiltered.status = fix->status;
    fixFiltered.latitude = fix->latitude;
    fixFiltered.longitude = fix->longitude;
    isNewFixAvailable = true;

    if (isnan(fix->latitude) || isnan(fix->longitude))
        return;

    geodeticToEnu(fix->latitude, fix->longitude, this->latestTangentX, this->latestTangentY, this->latestTangentZ);

    Kalman::Estimates estimates = this->filter.update({(double)this->latestAccelerationEast, (double)this->latestAccelerationNorth, (double)this->odomOffsetTheta},
                                                      {(double)this->latestTangentX, (double)this->latestTangentY, (double)this->latestYaw});
    //this->latestTangentX = estimates.x;
    //this->latestTangentY = estimates.y;

    long double tempLat, tempLon;
    enuToGeodetic(this->latestTangentX, this->latestTangentY, this->latestTangentZ, tempLat, tempLon);

    // There should be filtered GPS signal
    // but enuToGeodetic is not so accurate.
    fixFiltered.latitude = fix->latitude;
    fixFiltered.longitude = fix->longitude;

    // Fix updated so reset offsets.
    this->odomOffsetX = 0;
    this->odomOffsetY = 0;
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
    //tfTransformTangent.inverse();

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

const sensor_msgs::NavSatFix &FixToTf::getFilteredFixMsg()
{
    isNewFixAvailable = false;
    fixFiltered.header.stamp = ros::Time::now();
    return fixFiltered;
}