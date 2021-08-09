#include "../../include/gps_to_tf/kalman.hpp"
#include <iostream>

Kalman::Kalman(double _processVariance, double _measurementVariance)
    : processVariance(_processVariance),
      measurementVariance(_measurementVariance)
{
}

Kalman::Estimates Kalman::update(const Inputs &inputs, const Measurements &measurements)
{
    timeVar now = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - previousUpdateTime).count() / (double)1000.0;

    Eigen::Vector3d u;
    u << inputs.eastAcc, inputs.northAcc;

    Eigen::Vector3d z;
    z << measurements.x, measurements.y;

    Eigen::Matrix3d B;
    B << (double)0.5 * powf64(dt, 2), 0,
        0, (double)0.5 * powf64(dt, 2);

    Eigen::Matrix3d Q;
    Q << this->processVariance, 0,
        0, this->processVariance;

    Eigen::Matrix3d R;
    R << this->measurementVariance, 0,
        0, this->measurementVariance;

    if (this->fstScan)
    {
        this->fstScan = false;
        this->aPosteriori_xHat = z;
        this->aPosteriori_P = Q;
    }

    // Predict.
    Eigen::Vector3d aPriori_xHat = this->aPosteriori_xHat + B * u;
    Eigen::Matrix3d aPriori_P = this->aPosteriori_P + Q;

    // Update.
    Eigen::Vector3d innovation = z - aPriori_xHat;
    Eigen::Matrix3d innovationCovariance = aPriori_P + R;
    Eigen::Matrix3d K = aPriori_P * innovationCovariance.transpose();
    this->aPosteriori_xHat = aPriori_xHat + K * innovation;
    this->aPosteriori_P = (Eigen::Matrix3d::Identity() - K) * aPriori_P;

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