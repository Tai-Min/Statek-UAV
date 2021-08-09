#pragma once
#include <Eigen/Dense>
#include <chrono>

class Kalman
{
public:
    struct Inputs
    {
        double eastAcc;
        double northAcc;
    };

    struct Measurements{
        double x;
        double y;
    };

    typedef Measurements Estimates;
private:
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> timeVar;

    timeVar previousUpdateTime = std::chrono::high_resolution_clock::now();

    double processVariance;
    double measurementVariance;
    bool fstScan = true;
    Eigen::Vector3d aPosteriori_xHat;
    Eigen::Matrix3d aPosteriori_P;

public:
    Kalman(double _processVariance = 0.001, double _measurementVariance = 5);
    Estimates update(const Inputs &inputs, const Measurements& measurements);
    Estimates getEstimates() const;
};