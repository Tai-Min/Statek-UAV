#pragma once
#include <cppad/cppad.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>

class MPC
{
private:
    class FG_eval; //!< Class required for cost evaluation.

    /**
     * @brief contains information about one path segment.
     * Segment means a line with start and end coordinates.
     */
    struct PathSegment
    {
        double x0; //!< Start x of the segment.
        double y0; //!< Start y of the segment.
        double x1; //!< End x of the segment.
        double y1; //!< End y of the segment.

        double angle; //!< Angle in radians of the segment on local map from start to end coordinates.
    };

    typedef std::vector<PathSegment> Path; //!< Contains path as a vector of path segments.

public:
    /**
     * @brief Weights used in cost function.
     */
    struct CostWeights
    {
        double poseWeight;                 //!< How strong pose error in reference to closest path segment affects the cost function.
        double cteWeight;                  //!< How strong cross track error affects the cost function.
        double oeWeight;                   //!< How strong orientation error affects the cost function.
        double linearVelocityWeight;       //!< How important it is to minimize use of linear velocity control signal.
        double angularVelocityWeight;      //!< How important it is to minimize use of angular velocity control signal.
        double linearVelocityDeltaWeight;  //!< How important it is to smooth the change between linear velocity control signal (affects the smoothnes of movement).
        double angularVelocityDeltaWeight; //!< How important it is to smooth the change between angular velocity control signal (affects the smoothnes of movement).
    };

    /**
     * @brief Constraits for control variables.
     */
    struct Constraints
    {
        double linearMin;  //!< Max linear velocity in backward direction (preferably 0).
        double linearMax;  //!< Max linear velocity in forward direction.
        double angularMin; //!< Max angular velocity to the right.
        double angularMax; //!< Max angular velocity to the left.
    };

    /**
     * @brief Current state of the vehicle.
     */
    struct State
    {
        double x;   //!< X position in reference to local map.
        double y;   //!< Y position in reference to local map.
        double yaw; //!< Orientation of the robot in reference to local map.
        double cte; //!< Cross track error (distance of the vehicle to closest end of path segment).
        double oe;  //!< Orientation error (difference between desired and current orientations).
        int cteIdx; //!< Index of path segment closest to the vehicle.
    };

    /**
     * @brief Current actuation of the vehicle.
     */
    struct Inputs
    {
        double linearVelocity;  //!< In m/s.
        double angularVelocity; //!< In rad/s.
    };

    /**
     * @brief Output of the MPC.
     */
    struct Control
    {
        double linearVelocity;              //!< Linear velocity control variable in m/s.
        double angularVelocity;             //!< Angular velocity control variable in rad/s.
        std::vector<State> stateTrajectory; //!< Contains trajectory of the state at every sample.
        bool success;                       //!< Whether MPC succeeded.
    };

private:
    typedef CPPAD_TESTVECTOR(double) Dvector; //!< Required for Ipopt solver. Simple vector of doubles.

    const std::string solverOptions = "Integer print_level 0\nSparse true forward\nSparse true reverse\nNumeric max_cpu_time 10.0\n"; //!< Config for Ipopt solver.

    const double horizonSampling;                    //!< Horizon sampling time in seconds.
    const double horizonDuration;                    //!< Horizon duration in seconds.
    const int N = horizonDuration / horizonSampling; //!< Horizon duration in samples;

    const int numInputs = 2;                 //!< Number of inputs to the model (see MPC::Inputs).
    const int numVars = numInputs * (N - 1); //!< Number of variables for MPC. There are one less inputs as the last inputs are unused anyway (not used in last iteration of FG_eval()).

    const int linearVelocityStart = 0;                            //!< Index of first linear velocity input value in Dvectors and ADvectors.
    const int angularVelocityStart = linearVelocityStart + N - 1; //!< Index of first angular velocity input value in Dvectors and ADvectors. There is one less linearVelocityStart so subtract 1.*/

    const Constraints constraints; //!< Motion constraints.
    const CostWeights weights;     //!< Weights for cost function.

    Path path;              //!< Path received from ROS in form for better computing stuff such as cte and oe.
    Inputs inputs = {0, 0}; //!< Current actuations received from twist callback.

    /**
     * @brief Wrap given angle to 0 - 2PI.
     * @param angle Angle to wrap.
     * @return Current angle but in range of 0 - 2PI.
     */
    static double wrapToTwoPi(double angle);

    /**
     * @brief Compute closest distance from x0, y0 to line made of points x1, y1 and x2, y2.
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return Distance from x0, y0 to line.
     */
    static CppAD::AD<double> distancePointLine(CppAD::AD<double> x0, CppAD::AD<double> y0, CppAD::AD<double> x1, CppAD::AD<double> y1, CppAD::AD<double> x2, CppAD::AD<double> y2);

    /**
     * @brief Compute closest distance from x0, y0 to line made of points x1, y1 and x2, y2.
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return Distance from x0, y0 to line.
     */
    static double distancePointLine(double x0, double y0, double x1, double y1, double x2, double y2);

    /**
     * @brief Find cross track error and orientation error to the closest segment on the path.
     * @param x X position of the robot on local map.
     * @param y Y position of the robot on local map.
     * @param yaw Yaw of the robot in reference to local map.
     * @param path Path to check.
     * @param cte Found cross track error.
     * @param oe Found orientation error.
     * @return Index of path segment which is closest to the vehicle. Might return -1 if path is empty.
     */
    static int findCteOe(CppAD::AD<double> x, CppAD::AD<double> y, CppAD::AD<double> yaw, const Path &path, CppAD::AD<double> &cte, CppAD::AD<double> &oe);

    /**
     * @brief Find cross track error and orientation error to the closest segment on the path.
     * @param x X position of the robot on local map.
     * @param y Y position of the robot on local map.
     * @param yaw Yaw of the robot in reference to local map.
     * @param path Path to check.
     * @param cte Found cross track error.
     * @param oe Found orientation error.
     * @return Index of path segment which is closest to the vehicle. Might return -1 if path is empty.
     */
    static int findCteOe(double x, double y, double yaw, const Path &path, double &cte, double &oe);

    /**
     * @brief Get transform from target to source frame.
     * @param targetFrame Target frame.
     * @param sourceFrame Source frame.
     * @param ok Whether operation succeeded.
     * @return Requested transform.
     */
    static geometry_msgs::TransformStamped getTransform(const std::string &targetFrame, const std::string &sourceFrame, bool &ok);

public:
    /**
     * @brief Class constructor.
     * @param horizonDuration Prediction and control horizons duration in seconds.
     * @param horizonSampling Sampling time of horizonDuration in seconds.
     */
    MPC(const double _horizonDuration, const double _horizonSampling,
        const Constraints &_constraints, const CostWeights &_weights);

    /**
     * @brief Get new control values.
     * @return New actuation for the vehicle along with predicted trajectory.
     */
    Control update();

    /**
     * @brief Callback for short term path.
     * @param pathMsg Message with short term path.
     */
    void onNewPath(const nav_msgs::Path::ConstPtr &pathMsg);

    /**
     * @brief Callback for most recent actuations.
     * @param currentInputs Message with actuations.
     */
    void onNewInputs(const geometry_msgs::TwistStamped::ConstPtr &currentInputs);
};