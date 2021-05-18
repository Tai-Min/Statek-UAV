#pragma once

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
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

    const double northCompensation = 0;

    std::string mapFrame;   //!< Local map frame link.
    std::string earthFrame; //!< Earth frame link.

    double originPhi = 0;      //!< Longitude of origin of tangent plane in radians.
    double originLambda = 0;   //!< Latitude of origin of tangent plane in radians.
    double originEcefX = 0;    //!< ECEF coordinate of tangent plane in meters.
    double originEcefY = 0;    //!< ECEF coordinate of tangent plane in meters.
    double originEcefZ = 0;    //!< ECEF coordinate of tangent plane in meters.
    double latestTangentX = 0; //!< Latest position of robot in tangent plane.
    double latestTangentY = 0; //!< Latest position of robot in tangent plane.
    double latestYaw = 0;      //!< Latest rotation of robot in tangent plane.
    double latestVelocityNorth; //!< Latest velocity to the north (for Kalman filter).
    double latestVelocityEast; //!< Latest velocity to the east (for Kalman filter).

    // Odometry memory for compensation.
    double latestOdomX = 0;     //!< Latest odometry reading. Required to compute travelled distance since previous odom.
    double latestOdomY = 0;     //!< Latest odometry reading. Required to compute travelled distance since previous odom.
    
    // Odometry compensation.
    double odomOffsetX = 0;     //!< Accomodate movement in x direction between GPS updates.
    double odomOffsetY = 0;     //!< Accomodate movement in y direction between GPS updates.

    double sign(double v);

    /**
     * @brief Convert latitude and longitude to ECEF x, y, z coordinates.
     * @param lat Latitude.
     * @param lon longitude.
     * @param ecefX Result ECEF x.
     * @param ecefY Result ECEF y.
     * @param ecefZ Result ECEF z.
     */
    static void geodeticToEcef(double lat, double lon, double &ecefX, double &ecefY, double &ecefZ);

    /**
     * @brief Convert ECEF coordinates to latitude and longitude.
     * @param ecefX ECEF X.
     * @param ecefY ECEF Y.
     * @param ecefZ ECEF Z.
     * @param lat Result latitude.
     * @param lon Result longitude.
     */
    void EcefToGeodetic(double ecefX, double ecefY, double ecefZ, double &lat, double &lon);

    /**
     * @brief Convert ECEF coordinates to ENU.
     * @param ecefX ECEF x.
     * @param ecefY ECEF y.
     * @param ecefZ ECEF z
     * @param enuX Result ENU x.
     * @param enuY Result Enu y.
     */
    void EcefToEnu(double ecefX, double ecefY, double ecefZ, double &enuX, double &enuY);

    /**
     * @brief Convert ENU coordinates to ECEF.
     * @param enuX ENU X.
     * @param enuY ENU Y.
     * @param ecefX Result ECEF X.
     * @param ecefY Result ECEF Y.
     * @param ecefZ Result ECEF Z.
     */
    void EnuToEcef(double enuX, double enuY, double &ecefX, double &ecefY, double &ecefZ);

public:
    /**
     * @brief Class constructor.
     * @param originLat Latitude for the center of tangent plane.
     * @param originLon Longtitute of the center of tangent plane.
     * @param _northCompensation Angle (in radians) to compensate imu's reading to Y axis of transform will point to true north.
     * @param _mapFrame Map frame.
     * @param _earthFrame Earth frame.
     */
    FixToTf(double originLat, double originLon, double _northCompensation, std::string _mapFrame, std::string _earthFrame);

    /**
     * @brief Convert GPS coordinates to ENU.
     * @param lat Latitude.
     * @param lon longitude.
     * @param enuX ENU x.
     * @param enuY ENU y.
     */
    void geodeticToEnu(double lat, double lon, double &enuX, double &enuY);

    void enuToGeodetic(double enuX, double enuY, double &lat, double &lon);

    void geodeticToEnuService();

    void enuToGeodeticService();

    /**
     * @brief Callback for odom subscriber.
     * @param odom Odometry message.
     */
    void onNewOdom(const nav_msgs::Odometry::ConstPtr &odom);

    /**
     * @brief Callback for IMU subscriber.
     * Update rotation of the robot based on 9 DOF IMU.
     * @param imu IMU message.
     */
    void onNewImu(const sensor_msgs::Imu::ConstPtr &imu);

    /**
     * @brief Called on new GPS fix.
     * Update position of the robot based on GPS fix.
     * @param fix Fix.
     */
    void onNewFix(const sensor_msgs::NavSatFix::ConstPtr &fix);

    /**
     * @brief Get transformation from earth to local map link.
     * @param gpsToMap Transformation from GPS device link to local map.
     * @return Transformation from origin of earth's tangent plane to the robot.
     */
    geometry_msgs::TransformStamped getTransformMsg(const geometry_msgs::Transform &gpsToMap);
};