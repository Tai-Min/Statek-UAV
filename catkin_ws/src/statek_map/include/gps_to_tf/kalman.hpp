#pragma once
#include <Eigen/Dense>
#include <chrono>
#include <vector>

class Kalman
{
public:
    struct Inputs
    {
        double eastAcc;
        double northAcc;
    };

    struct Measurements
    {
        double x;
        double y;
    };

    typedef Measurements Estimates;

private:
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> timeVar;

    timeVar previousUpdateTime = std::chrono::high_resolution_clock::now();

    const std::vector<double> processVariance;
    const std::vector<double> measurementVariance;

    bool fstScan = true;
    Eigen::Vector2d aPosteriori_xHat;
    Eigen::Matrix2d aPosteriori_P;

public:
    Kalman(const std::vector<double> &_processVariance, const std::vector<double> &_measurementVariance);
    Estimates update(const Inputs &inputs, const Measurements &measurements);
    Estimates getEstimates() const;
};