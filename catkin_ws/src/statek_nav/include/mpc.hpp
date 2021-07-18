#pragma once
#include <cppad/cppad.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <thread>
#include <mutex>

class MPC
{
private:
    class FG_eval;                            //!< Class required for cost evaluation.
    typedef CPPAD_TESTVECTOR(double) Dvector; //!< Required for Ipopt solver. Simple vector of doubles.

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
        bool success;
    };

private:
    void tfThreadFcn();

    std::atomic_bool stopTfThread = {false}; //!< Flag to stop tf thread.
    std::atomic_bool tfReceived = {false};   //!< Flag to indicate that some tf was received and MPC can work.
    mutable std::mutex stateMtx;             //!< Lock for state variable.
    std::thread tfThread;                    //!< Thread to receive tf from map to footprint as it's takes around 100ms.

    const std::string solverOptions; //!< Config for Ipopt solver.
    const double goalArea;           //!< Radius of area around the goal point which is considered as a goal. Used to select next goal even if the vehicle isn't precisely on goal point.

    const double horizonSamplingForward; //!< Horizon sampling time when vehicle moves forward. In seconds.
    const double horizonDurationForward; //!< Horizon duration when vehicle moves forward. In seconds.
    const double horizonSamplingRotate;  //!< Horizon sampling time when vehicle rotates from abs(oe) = PI. In seconds.
    const double horizonDurationRotate;  //!< Horizon duration when vehicle rotates from abs(oe) = PI. In seconds.

    double horizonSampling; //!< In seconds. Will change depending on oe.
    double horizonDuration; //!< In seconds. Will change depending on oe.
    int N;                  //!< Horizon duration in samples. Will change depending on oe.

    const int numInputs = 2; //!< Number of inputs to the model (see MPC::Inputs).

    const int linearVelocityStart = 0;                      //!< Index of first linear velocity input value in Dvectors and ADvectors.
    int angularVelocityStart = linearVelocityStart + N - 1; //!< Index of first angular velocity input value in Dvectors and ADvectors. There is one less linearVelocityStart so subtract 1.*/

    const Constraints constraints; //!< Motion constraints.
    const CostWeights weights;     //!< Weights for cost function.

    Path path;                        //!< Path received from ROS in form for better computing stuff such as cte and oe.
    State state = {0, 0, 0, 0, 0, 0}; //!< Current state received from tf.
    Inputs inputs = {0, 0};           //!< Current actuations received from twist callback.

    State onMapState = {0, 0, 0, 0, 0};

    /**
     * @brief Wrap given angle to -PI to PI.
     * @param angle Angle to wrap.
     * @return Current angle but in range of -PI to PI.
     */
    static double wrapToPi(double angle);

    /**
     * @brief Compute closest distance from x0, y0 to line made of points x1, y1 and x2, y2.
     * @param x0 X of the vehicle.
     * @param y0 Y of the vehicle.
     * @param x1 X start of the line.
     * @param y1 Y start of the line.
     * @param x2 X end of the line.
     * @param y2 Y end of the line.
     * @return Distance from x0, y0 to line.
     */
    static double distancePointLine(double x0, double y0, double x1, double y1, double x2, double y2);

    /**
     * @brief Find cross track error and orientation error to the closest segment on the path*.
     * * If vehicle is too close to the end of the closest segment, next segment is used.
     * @param state State of the vehicle with x, y, yaw filled. cte and oe fields will be filled inside of this function.
     * @param lastIdx Previous found index. Use 0 if there is no previous index or it's unknown.
     * @return Index of path segment which is closest to the vehicle. Might return -1 if path is empty.
     */
    void findCteOe(State &_state, int lastIdx) const;

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
     * @param horizonDurationForward Prediction and control horizons duration when vehicle moves forward. In seconds.
     * @param horizonSamplingForward Sampling time of horizonDuration when vehicle moves forward. In seconds.
     * @param horizonDurationRotate Prediction and control horizons duration when vehicle rotates from abs(oe) = PI. In seconds.
     * @param horizonSamplingRotate Sampling time of horizonDuration when vehicle rotates from abs(oe) = PI. In seconds.
     * @param constraints MPC motion constraints.
     * @param weights Weights for cost function.
     * @param goalArea Area around goal point which is treated as the goal itself.
     * @param solverOptions Options for Ipopt solver. Each option in new line.
     */
    MPC(double _horizonDurationForward, double _horizonSamplingForward, double _horizonDurationRotate, double _horizonSamplingRotate,
        const Constraints &_constraints, const CostWeights &_weights, const double _goalArea = 0.1, const std::string &_solverOptions = "");

    ~MPC();

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

/**
 * @brief Cost function for differential drive under motion constraints.
 */
class MPC::FG_eval
{
public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector; //!< Required for Ipopt solver. Simple vector of AD doubles.

private:
    const MPC *parent = nullptr; //!< Parent of this object.

public:
    /**
     * @brief Class constructor.
     * @param _parent Parent object of this eval function.
     */
    FG_eval(const MPC *_parent);

    /**
     * @brief Evaluate found controls on doubles rather than AD variables.
     * @param controls Controls to use during evaluation.
     * @param initialState Initial state to use.
     * @return Trajectory of the state.
     */
    std::vector<MPC::State> evalControl(MPC::Dvector controls, const MPC::State &initialState) const;

    /**
     * @brief Compute closest distance from x0, y0 to line made of points x1, y1 and x2, y2.
     * @param x0 X of the vehicle.
     * @param y0 Y of the vehicle.
     * @param x1 X start of the line.
     * @param y1 Y start of the line.
     * @param x2 X end of the line.
     * @param y2 Y end of the line.
     * @return Distance from x0, y0 to line.
     */
    CppAD::AD<double> distancePointLine(CppAD::AD<double> x0, CppAD::AD<double> y0,
                                        CppAD::AD<double> x1, CppAD::AD<double> y1,
                                        CppAD::AD<double> x2, CppAD::AD<double> y2) const;

    /**
     * @brief Wrap given angle to -PI to PI.
     * @param angle Angle to wrap.
     * @return Current angle but in range of -PI to PI.
     */
    static CppAD::AD<double> wrapToPi(CppAD::AD<double> angle);

    /**
     * @brief Find cross track error and orientation error to the closest segment on the path*.
     * * If vehicle is too close to the end of the closest segment, next segment is used.
     * @param x X position of the robot on local map.
     * @param y Y position of the robot on local map.
     * @param yaw Yaw of the robot in reference to local map.
     * @param cte Found cross track error.
     * @param oe Found orientation error.
     * @return Index of path segment which is closest to the vehicle. Might return -1 if path is empty.
     */
    void findCteOe(CppAD::AD<double> x, CppAD::AD<double> y, CppAD::AD<double> yaw,
                   CppAD::AD<double> &cte, CppAD::AD<double> &oe,
                   CppAD::AD<double> &endX, CppAD::AD<double> &endY) const;

    /**
     * @brief Calculate cost of control under given 
     * @param fg Cost function and constraint functions.
     * @param vars Current control variables.
     */
    void operator()(ADvector &fg, const ADvector &vars) const;
};