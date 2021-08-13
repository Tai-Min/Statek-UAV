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

    double processVarianceNorth, processVarianceEast;
    double measurementVarianceNorth, measurementVarianceEast;
    bool fstScan = true;
    Eigen::Vector2d aPosteriori_xHat;
    Eigen::Matrix2d aPosteriori_P;

public:
    Kalman(double _processVarianceNorth, double _processVarianceEast, double _measurementVarianceNorth, double _measurementVarianceEast);
    Estimates update(const Inputs &inputs, const Measurements& measurements);
    Estimates getEstimates() const;
};