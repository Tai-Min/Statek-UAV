#include "../include/mpc.hpp"
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

#include <chrono>
#include <iostream>
using namespace std;

MPC::MPC(double _horizonDurationForward, double _horizonSamplingForward,
         double _horizonDurationRotate, double _horizonSamplingRotate,
         const Constraints &_constraints, const CostWeights &_weights,
         const double _goalArea, const std::string &_solverOptions)
    : horizonSamplingForward(_horizonSamplingForward), horizonDurationForward(_horizonDurationForward),
      horizonSamplingRotate(_horizonSamplingRotate), horizonDurationRotate(_horizonDurationRotate),
      constraints(_constraints), weights(_weights),
      goalArea(_goalArea), solverOptions(_solverOptions), tfThread(&MPC::tfThreadFcn, this)
{
}

MPC::~MPC()
{
    stopTfThread = true;
    tfThread.join();
}

void MPC::tfThreadFcn()
{
    auto previousTfTime = std::chrono::steady_clock::now();

    while (!stopTfThread)
    {
        auto now = std::chrono::steady_clock::now();

        // Check for tf timeouts to stop the controller.
        auto tfElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - previousTfTime).count();
        if (tfElapsed > 3000)
            tfReceived = false;

        // Get location of the robot on local map.
        bool ok = false;
        geometry_msgs::TransformStamped robotTransform = getTransform("statek/earth", "statek/base_footprint", ok);

        if (ok)
        {
            tfReceived = true; // Tf received so allow MPC to work.
            previousTfTime = std::chrono::steady_clock::now();

            // Convert orientation to tf quaternion.
            tf::Quaternion quat;
            tf::quaternionMsgToTF(robotTransform.transform.rotation, quat);

            // Convert orientation to RPY angles.
            double temp1, temp2, theta;
            tf::Matrix3x3(quat).getRPY(temp1, temp2, theta);
            theta = wrapToPi(theta);

            // Set current state.
            const std::lock_guard<std::mutex> lck(stateMtx);
            state = {robotTransform.transform.translation.x,
                     robotTransform.transform.translation.y,
                     theta, 0, 0, -1};

            // Get cross track error and orientation error.
            findCteOe(state, 0);
        }
    }
}

double MPC::wrapToPi(double angle)
{
    // Whiles because fmod works strange?
    if (angle > M_PI)
        angle -= M_PI;
    if (angle < -M_PI)
        angle += M_PI;
    if (angle == -M_PI)
        angle = M_PI;

    return angle;
}

double MPC::distancePointLine(double x0, double y0, double x1, double y1, double x2, double y2)
{
    return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void MPC::findCteOe(State &_state, int lastIdx) const
{
    int bestIdx = -1;
    _state.cte = std::numeric_limits<double>::max();

    for (int i = lastIdx; i < path.size(); i++)
    {
        double currentCte = distancePointLine(_state.x, _state.y, path[i].x0, path[i].y0, path[i].x1, path[i].y1);
        if (currentCte < _state.cte)
        {
            _state.cte = currentCte;
            bestIdx = i;
        }
    }

    if (bestIdx >= 0)
    {
        double distanceToEnd = sqrt(pow(_state.x - path[bestIdx].x1, 2) + pow(_state.y - path[bestIdx].y1, 2));

        // If distance of the vehicle and end of best segment is less than some limiting value
        // then we'd rather care for next segment if there is one.
        if (distanceToEnd < goalArea &&
            bestIdx < path.size() - 1)
        {
            bestIdx++;
            _state.cte = distancePointLine(_state.x, _state.y, path[bestIdx].x0, path[bestIdx].y0, path[bestIdx].x1, path[bestIdx].y1);
        }

        _state.oe = wrapToPi(path[bestIdx].angle - _state.yaw);
    }

    _state.cteIdx = bestIdx;
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
    auto startFcn = chrono::steady_clock::now();

    if (!tfReceived || !path.size())
        return {0, 0, {}, true};

    {
        const std::lock_guard<std::mutex> lckOe(stateMtx);

        // Calculace current prediction horizon depending on oe.
        // Use simple linear equation to do that.
        horizonSampling = abs(state.oe) * (horizonSamplingRotate - horizonSamplingForward) / M_PI + horizonSamplingForward;
        horizonDuration = abs(state.oe) * (horizonDurationRotate - horizonDurationForward) / M_PI + horizonDurationForward;
        N = horizonDuration / horizonSampling;
    }

    int numVars = numInputs * (N - 1);
    angularVelocityStart = linearVelocityStart + N - 1;

    // Initial state of variables.
    Dvector vars(numVars);
    for (int i = 0; i < numVars; i++)
    {
        vars[i] = 0;
    }

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
    FG_eval evalFcn(this);
    CppAD::ipopt::solve_result<Dvector> solution;

    const std::lock_guard<std::mutex> lckOpti(stateMtx);
    try
    {
        CppAD::ipopt::solve<Dvector, FG_eval>(
            solverOptions,
            vars, varsLowerConstraints, varsUpperConstraints,
            varsLowerConstraints, varsUpperConstraints,
            evalFcn, solution);
    }
    catch (...)
    {
        return {0, 0, {}, true};
    }

    /*if (solution.status != CppAD::ipopt::solve_result<Dvector>::success)
    {
        return {0, 0, {}, true};
    }*/

    Control control;
    control.success = true;

    // Apply first set of controls.
    control.linearVelocity = solution.x[linearVelocityStart];
    control.angularVelocity = solution.x[angularVelocityStart];

    // Copy state trajectory.
    control.stateTrajectory = evalFcn.evalControl(solution.x, state);

    auto endFcn = chrono::steady_clock::now();
    //ROS_WARN("MPC update elapsed time in milliseconds: %ldms.\n---", chrono::duration_cast<chrono::milliseconds>(endFcn - startFcn).count());

    return control;
}

void MPC::onNewPath(const nav_msgs::Path::ConstPtr &pathMsg)
{
    if (!pathMsg->poses.size())
        return;

    // If previous and current paths were not a straight line
    // but a bunch of segments, then check whether it is necessary to update it.
    // This prevents vehicle from being stuck in front of an obstacle
    // when map changes rapidly fron one side of this obstacle to another.
    if (path.size() > 1 && pathMsg->poses.size() > 2)
    {
        double distanceToGoal;
        double distanceTravelled;
        {
            const std::lock_guard<std::mutex> lckOpti(stateMtx);
            distanceToGoal = sqrt(pow(path[0].x1 - state.x, 2) + pow(path[0].y1 - state.y, 2));
            distanceTravelled = sqrt(pow(onMapState.x - state.x, 2) + pow(onMapState.y - state.y, 2));
        }

        // Ignore path if the vehicle haven't achieved subgoal
        // or travelled a bit or some time passed.
        if (distanceToGoal > goalArea || distanceTravelled < 0.2)
            return;
    }

    path.clear();

    // Translate path message into Path data type.
    for (int i = 0; i < pathMsg->poses.size() - 1; i++)
    {
        PathSegment segment;

        segment.x0 = pathMsg->poses[i].pose.position.x;
        segment.y0 = pathMsg->poses[i].pose.position.y;

        segment.x1 = pathMsg->poses[i + 1].pose.position.x;
        segment.y1 = pathMsg->poses[i + 1].pose.position.y;

        segment.angle = wrapToPi(atan2(segment.y1 - segment.y0, segment.x1 - segment.x0));

        path.push_back(segment);
    }

    const std::lock_guard<std::mutex> lckOpti(stateMtx);
    onMapState = state;
}

void MPC::onNewInputs(const geometry_msgs::TwistStamped::ConstPtr &currentInputs)
{
    inputs.linearVelocity = currentInputs->twist.linear.x;
    inputs.angularVelocity = currentInputs->twist.angular.z;
}
