#pragma once
#include <Eigen/Dense>

class MPC
{

    struct MPCResult {
        double linear;
        double angular;
    };

public:
    void setConstraints();
    void update();
};