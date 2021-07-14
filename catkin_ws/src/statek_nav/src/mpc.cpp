#include "../include/mpc.hpp"
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

#include <iostream>
using namespace std;

namespace
{
    bool isEqual(double a, double b, double eps)
    {
        return std::fabs(a - b) < eps;
    }
};

/**
 * @brief Cost function (f) for differential drive under motion constraints (g).
 */
class MPC::FG_eval
{
public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

private:
    const MPC *parent = nullptr; //!< Parent of this object.
    const MPC::State &state;     //!< Reference to initial state passed during object creation.

public:
    ADvector xTrajectory;
    ADvector yTrajectory;
    ADvector yawTrajectory;
    ADvector cteTrajectory;
    ADvector oeTrajectory;
    std::vector<int> cteIdx;

    /**
     * @brief Class constructor.
     */
    FG_eval(const MPC *_parent, const MPC::State &_state)
        : parent(_parent), xTrajectory(parent->N), yTrajectory(parent->N), yawTrajectory(parent->N),
          cteTrajectory(parent->N), oeTrajectory(parent->N), cteIdx(parent->N), state(_state) {}

    /**
     * @brief Calculate cost of control under given 
     * @param fg
     * @param vars
     */
    void operator()(ADvector &fg, const ADvector &vars)
    {
        // Create trajectory based on predicted controls.
        xTrajectory[0] = state.x;
        yTrajectory[0] = state.y;
        yawTrajectory[0] = state.yaw;
        cteTrajectory[0] = state.cte;
        oeTrajectory[0] = state.oe;
        cteIdx[0] = state.cteIdx;

        for (int i = 1; i < parent->N; i++)
        {
            xTrajectory[i] = xTrajectory[i - 1] + vars[parent->linearVelocityStart + i - 1] * CppAD::cos(yawTrajectory[i - 1]) * parent->horizonSampling;
            yTrajectory[i] = yTrajectory[i - 1] + vars[0] * CppAD::sin(yawTrajectory[parent->linearVelocityStart + i - 1]) * parent->horizonSampling;
            yawTrajectory[i] = yawTrajectory[i - 1] + vars[parent->angularVelocityStart + i - 1] * parent->horizonSampling;

            CppAD::AD<double> cte, oe;

            cteIdx[i] = findCteOe(xTrajectory[i], yTrajectory[i], yawTrajectory[i], parent->path, cte, oe);

            cteTrajectory[i] = cte;
            oeTrajectory[i] = oe;
        }

        fg[0] = 0; // Cost function.

        for (int i = 0; i < parent->N; i++)
        {
            // Minimize control error.
            fg[0] += parent->weights.poseWeight * (CppAD::pow(parent->path[cteIdx[i]].x1 - xTrajectory[i], 2) +
                                                   CppAD::pow(parent->path[cteIdx[i]].y1 - yTrajectory[i], 2));
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
};

MPC::MPC(const double _horizonDuration, const double _horizonSampling,
         const Constraints &_constraints, const CostWeights &_weights)
    : horizonSampling(_horizonSampling), horizonDuration(_horizonDuration),
      constraints(_constraints), weights(_weights)
{
}

double MPC::wrapToTwoPi(double angle)
{
    if (angle < 0)
        return fmod(angle, 2 * M_PI);
    return angle;
}

CppAD::AD<double> MPC::distancePointLine(CppAD::AD<double> x0, CppAD::AD<double> y0, CppAD::AD<double> x1, CppAD::AD<double> y1, CppAD::AD<double> x2, CppAD::AD<double> y2)
{
    return CppAD::abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / CppAD::sqrt(pow(x2 - x1, 2) + CppAD::pow(y2 - y1, 2));
}

double MPC::distancePointLine(double x0, double y0, double x1, double y1, double x2, double y2)
{
    return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int MPC::findCteOe(CppAD::AD<double> x, CppAD::AD<double> y, CppAD::AD<double> yaw, const Path &path, CppAD::AD<double> &cte, CppAD::AD<double> &oe)
{
    int bestIdx = -1;
    CppAD::AD<double> bestCte = std::numeric_limits<double>::max();

    for (int i = 0; i < path.size(); i++)
    {
        CppAD::AD<double> currentCte = distancePointLine(x, y, path[i].x0, path[i].y0, path[i].x1, path[i].y1);
        if (currentCte < bestCte)
        {
            bestCte = currentCte;
            bestIdx = i;
        }
    }

    cte = bestCte;

    if (bestIdx >= 0)
        oe = path[bestIdx].angle - yaw;

    return bestIdx;
}

int MPC::findCteOe(double x, double y, double yaw, const Path &path, double &cte, double &oe)
{
    int bestIdx = -1;
    double bestCte = std::numeric_limits<double>::max();

    for (int i = 0; i < path.size(); i++)
    {
        double currentCte = distancePointLine(x, y, path[i].x0, path[i].y0, path[i].x1, path[i].y1);
        if (currentCte < bestCte)
        {
            bestCte = currentCte;
            bestIdx = i;
        }
    }

    cte = bestCte;

    if (bestIdx >= 0)
        oe = path[bestIdx].angle - yaw;

    return bestIdx;
}

geometry_msgs::TransformStamped MPC::getTransform(const std::string &targetFrame, const std::string &sourceFrame, bool &ok)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener transformListener(tfBuffer);

    geometry_msgs::TransformStamped result;

    if (targetFrame == "" || sourceFrame == "")
    {
        ok = false;
        return result;
    }

    // Fixes extrapolation into the future in Rviz if run remotely.
    ros::Time now = ros::Time::now();
    ok = tfBuffer.canTransform(targetFrame, sourceFrame, now, ros::Duration(10.0));
    if (!ok)
        return result;

    ok = true;
    try
    {
        result = tfBuffer.lookupTransform(targetFrame, sourceFrame, now);
    }
    catch (tf::TransformException ex)
    {
        ok = false;
    }

    return result;
}

MPC::Control MPC::update()
{
    // Empty path, nothing to follow.
    if (!path.size())
    {
        return {0, 0, {}, false};
    }

    // Get location of the robot on local map.
    bool ok = false;
    geometry_msgs::TransformStamped robotTransform = getTransform("statek/map/local_map_link", "statek/base_footprint", ok);

    if (!ok)
    {
        return {0, 0, {}, false};
    }

    // Convert orientation to tf quaternion.
    tf::Quaternion quat;
    tf::quaternionMsgToTF(robotTransform.transform.rotation, quat);

    // Convert orientation to RPY angles.
    double temp1, temp2, theta;
    tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);
    theta = wrapToTwoPi(theta);

    // Get cross track error and orientation error.
    double cte;
    double oe;

    int idx = findCteOe(robotTransform.transform.translation.x,
                        robotTransform.transform.translation.y,
                        theta,
                        path,
                        cte,
                        oe);

    // Set current state.
    const State state = {robotTransform.transform.translation.x,
                         robotTransform.transform.translation.y,
                         theta, cte, oe, idx};

    // Initial state of variables.
    Dvector vars(numVars);
    for (int i = 0; i < numVars; i++)
    {
        vars[i] = 0;
    }

    // First actuations must be set to current actuations from twist callback.
    vars[linearVelocityStart] = inputs.linearVelocity;
    vars[angularVelocityStart] = inputs.angularVelocity;

    // Constraints for variables.
    Dvector varsLowerConstraints(numVars);
    Dvector varsUpperConstraints(numVars);
    for (int i = linearVelocityStart; i < angularVelocityStart; i++)
    {
        varsLowerConstraints[i] = constraints.linearMin;
        varsUpperConstraints[i] = constraints.linearMax;
    }
    for (int i = angularVelocityStart; i < numVars; i++)
    {
        varsLowerConstraints[i] = constraints.angularMin;
        varsUpperConstraints[i] = constraints.angularMax;
    }

    // Constraints for constraint equations g(x).
    Dvector stateLowerConstraints(0);
    Dvector stateUpperConstraints(0);

    // MPC evaluation.
    FG_eval evalFcn(this, state);
    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve<Dvector, FG_eval>(
        solverOptions,
        vars, varsLowerConstraints, varsUpperConstraints,
        varsLowerConstraints, varsUpperConstraints,
        evalFcn, solution);

    cout << "Cost: " << solution.obj_value << endl;

    if (solution.status != CppAD::ipopt::solve_result<Dvector>::success)
        return {0, 0, {}, false};

    Control control;
    control.success = true;

    // Apply first set of controls.
    control.linearVelocity = solution.x[linearVelocityStart];
    control.angularVelocity = solution.x[angularVelocityStart];

    // Copy state trajectory.
    for (int i = 0; i < N; i++)
    {
        State s = {
            CppAD::Value(evalFcn.xTrajectory[i]),
            CppAD::Value(evalFcn.yTrajectory[i]),
            CppAD::Value(evalFcn.yawTrajectory[i]),
            CppAD::Value(evalFcn.cteTrajectory[i]),
            CppAD::Value(evalFcn.oeTrajectory[i]),
        };

        control.stateTrajectory.push_back(s);
    }

    return control;
}

void MPC::onNewPath(const nav_msgs::Path::ConstPtr &pathMsg)
{
    path.clear();

    // Translate path message into Path data type.
    for (int i = 0; i < pathMsg->poses.size() - 1; i++)
    {
        PathSegment segment;

        segment.x0 = pathMsg->poses[i].pose.position.x;
        segment.y0 = pathMsg->poses[i].pose.position.y;

        segment.x1 = pathMsg->poses[i + 1].pose.position.x;
        segment.y1 = pathMsg->poses[i + 1].pose.position.y;

        segment.angle = wrapToTwoPi(atan2(segment.y1 - segment.y0, segment.x1 - segment.x0));

        cout << "Angle " << i << ": " << segment.angle << endl;

        path.push_back(segment);
    }
}

void MPC::onNewInputs(const geometry_msgs::TwistStamped::ConstPtr &currentInputs)
{
    inputs.linearVelocity = currentInputs->twist.linear.x;
    inputs.angularVelocity = currentInputs->twist.angular.z;
}