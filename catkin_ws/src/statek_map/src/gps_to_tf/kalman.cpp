#include "../../include/gps_to_tf/kalman.hpp"
#include <iostream>

Kalman::Kalman(double _processVarianceNorth, double _processVarianceEast, double _measurementVarianceNorth, double _measurementVarianceEast)
    : processVarianceNorth(_processVarianceNorth),
      processVarianceEast(_processVarianceEast),
      measurementVarianceNorth(_measurementVarianceNorth),
      measurementVarianceEast(_measurementVarianceEast)
{
}

Kalman::Estimates Kalman::update(const Inputs &inputs, const Measurements &measurements)
{
    timeVar now = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - previousUpdateTime).count() / (double)1000.0;

    Eigen::Vector2d u;
    u << inputs.eastAcc, inputs.northAcc;

    Eigen::Vector2d z;
    z << measurements.x, measurements.y;

    Eigen::Matrix2d B;
    B << (double)0.5 * powf64(dt, 2), 0,
        0, (double)0.5 * powf64(dt, 2);

    Eigen::Matrix2d Q;
    Q << this->processVarianceEast, 0,
        0, this->processVarianceNorth;

    Eigen::Matrix2d R;
    R << this->measurementVarianceEast, 0,
        0, this->measurementVarianceNorth;

    if (this->fstScan)
    {
        this->fstScan = false;
        this->aPosteriori_xHat = z;
        this->aPosteriori_P = Q;
    }

    // Predict.
    Eigen::Vector2d aPriori_xHat = this->aPosteriori_xHat + B * u;
    Eigen::Matrix2d aPriori_P = this->aPosteriori_P + Q;

    // Update.
    Eigen::Vector2d innovation = z - aPriori_xHat;
    Eigen::Matrix2d innovationCovariance = aPriori_P + R;
    Eigen::Matrix2d K = aPriori_P * innovationCovariance.transpose();
    this->aPosteriori_xHat = aPriori_xHat + K * innovation;
    this->aPosteriori_P = (Eigen::Matrix2d::Identity() - K) * aPriori_P;

    previousUpdateTime = now;

    return this->getEstimates();
}

Kalman::Estimates Kalman::getEstimates() const
{
    Estimates e;
    e.x = this->aPosteriori_xHat(0);
    e.y = this->aPosteriori_xHat(1);
    return e;
}