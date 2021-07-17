#include "../include/mpc.hpp"

MPC::FG_eval::FG_eval(const MPC *_parent)
    : parent(_parent){}

CppAD::AD<double> MPC::FG_eval::distancePointLine(CppAD::AD<double> x0, CppAD::AD<double> y0,
                                                  CppAD::AD<double> x1, CppAD::AD<double> y1,
                                                  CppAD::AD<double> x2, CppAD::AD<double> y2) const
{
    return CppAD::abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / CppAD::sqrt(pow(x2 - x1, 2) + CppAD::pow(y2 - y1, 2));
}

std::vector<MPC::State> MPC::FG_eval::evalControl(MPC::Dvector controls, const MPC::State &initialState) const
{
    MPC::State previousState = initialState;
    std::vector<MPC::State> trajectory;

    trajectory.push_back(initialState);

    for (int i = 1; i < parent->N; i++)
    {
        MPC::State newState;

        newState.x = previousState.x + controls[parent->linearVelocityStart + i - 1] * cos(previousState.yaw) * parent->horizonSampling;
        newState.y = previousState.y + controls[parent->linearVelocityStart + i - 1] * sin(previousState.yaw) * parent->horizonSampling;
        newState.yaw = previousState.yaw + controls[parent->angularVelocityStart + i - 1] * parent->horizonSampling;

        parent->findCteOe(newState, previousState.cteIdx);

        previousState = newState;

        trajectory.push_back(newState);
    }

    return trajectory;
}

void MPC::FG_eval::findCteOe(CppAD::AD<double> x, CppAD::AD<double> y, CppAD::AD<double> yaw,
                             CppAD::AD<double> &cte, CppAD::AD<double> &oe,
                             CppAD::AD<double> &endX, CppAD::AD<double> &endY) const
{
    ADvector currentCtes(parent->path.size());
    ADvector distancesToEnd(parent->path.size());

    ADvector ctes(parent->path.size() + 1);
    ADvector oes(parent->path.size() + 1);
    ADvector endXs(parent->path.size() + 1);
    ADvector endYs(parent->path.size() + 1);

    ctes[0] = CppAD::numeric_limits<CppAD::AD<double>>::max();
    oes[0] = CppAD::numeric_limits<CppAD::AD<double>>::max();
    endXs[0] = CppAD::numeric_limits<CppAD::AD<double>>::max();
    endYs[0] = CppAD::numeric_limits<CppAD::AD<double>>::max();

    for (int i = 0; i < parent->path.size(); i++)
    {
        distancesToEnd[i] = CppAD::sqrt(CppAD::pow(x - parent->path[i].x1, 2) + CppAD::pow(y - parent->path[i].y1, 2));

        // Too close to end the end of the segment
        // so simply ignore it and find next segment.
        if (distancesToEnd[i] > parent->goalArea)
        {
            currentCtes[i] = distancePointLine(x, y, parent->path[i].x0, parent->path[i].y0, parent->path[i].x1, parent->path[i].y1);

            ctes[i + 1] = CppAD::CondExpLt(currentCtes[i], ctes[i], currentCtes[i], ctes[i]);
            oes[i + 1] = CppAD::CondExpLt(currentCtes[i], ctes[i], CppAD::AD<double>(parent->path[i].angle - yaw), oes[i]);
            endXs[i + 1] = CppAD::CondExpLt(currentCtes[i], ctes[i], CppAD::AD<double>(parent->path[i].x1), endXs[i]);
            endYs[i + 1] = CppAD::CondExpLt(currentCtes[i], ctes[i], CppAD::AD<double>(parent->path[i].y1), endYs[i]);
        }
        else
        {
            ctes[i + 1] = CppAD::numeric_limits<CppAD::AD<double>>::max();
            oes[i + 1] = CppAD::numeric_limits<CppAD::AD<double>>::max();
            endXs[i + 1] = CppAD::numeric_limits<CppAD::AD<double>>::max();
            endYs[i + 1] = CppAD::numeric_limits<CppAD::AD<double>>::max();
        }
    }

    cte = ctes[parent->path.size()];
    oe = oes[parent->path.size()];
    endX = endXs[parent->path.size()];
    endY = endYs[parent->path.size()];
}

void MPC::FG_eval::operator()(ADvector &fg, const ADvector &vars) const
{
    // Trajectory vectors.
    ADvector xTrajectory(parent->N);
    ADvector yTrajectory(parent->N);
    ADvector yawTrajectory(parent->N);
    ADvector cteTrajectory(parent->N);
    ADvector oeTrajectory(parent->N);

    // Those trajectories contain end X and Y coordinates of the segment with best cte
    // that was found using findCteOe function.
    // Those are used as goal during cost error minimalization and provide that the robot
    // will move forward along the path.
    ADvector endXTrajectory(parent->N);
    ADvector endYTrajectory(parent->N);

    // Copy current state as first sample of trajectory.
    xTrajectory[0] = parent->state.x;
    yTrajectory[0] = parent->state.y;
    yawTrajectory[0] = parent->state.yaw;
    cteTrajectory[0] = parent->state.cte;
    oeTrajectory[0] = parent->state.oe;
    endXTrajectory[0] = parent->path[0].x1;
    endYTrajectory[0] = parent->path[0].y1;

    // Create trajectory based on predicted controls.
    for (int i = 1; i < parent->N; i++)
    {
        xTrajectory[i] = xTrajectory[i - 1] + vars[parent->linearVelocityStart + i - 1] * CppAD::cos(yawTrajectory[i - 1]) * parent->horizonSampling;
        yTrajectory[i] = yTrajectory[i - 1] + vars[parent->linearVelocityStart + i - 1] * CppAD::sin(yawTrajectory[i - 1]) * parent->horizonSampling;
        yawTrajectory[i] = yawTrajectory[i - 1] + vars[parent->angularVelocityStart + i - 1] * parent->horizonSampling;
        findCteOe(xTrajectory[i], yTrajectory[i], yawTrajectory[i],
                  cteTrajectory[i], oeTrajectory[i],
                  endXTrajectory[i], endYTrajectory[i]);
    }

    fg[0] = 0; // Cost function.

    // Minimize control error.
    for (int i = 0; i < parent->N; i++)
    {
        fg[0] += parent->weights.poseWeight * (CppAD::pow(endXTrajectory[i] - xTrajectory[i], 2) +
                                               CppAD::pow(endYTrajectory[i] - yTrajectory[i], 2));
        fg[0] += parent->weights.cteWeight * CppAD::pow(cteTrajectory[i], 2);
        fg[0] += parent->weights.oeWeight * CppAD::pow(oeTrajectory[i], 2);
    }

    // Minimize use of actuators.
    for (int i = 0; i < parent->N - 1; i++)
    {
        fg[0] += parent->weights.linearVelocityWeight * CppAD::pow(vars[parent->linearVelocityStart + i], 2);
        fg[0] += parent->weights.angularVelocityWeight * CppAD::pow(vars[parent->angularVelocityStart + i], 2);
    }

    // Minimize gap between sequential actuations for smoothness of movement.
    for (int i = 1; i < parent->N - 1; i++)
    {
        fg[0] += parent->weights.linearVelocityDeltaWeight * (CppAD::pow(vars[parent->linearVelocityStart + i] - vars[parent->linearVelocityStart + i - 1], 2));
        fg[0] += parent->weights.angularVelocityDeltaWeight * (CppAD::pow(vars[parent->angularVelocityStart + i] - vars[parent->angularVelocityStart + i - 1], 2));
    }
}