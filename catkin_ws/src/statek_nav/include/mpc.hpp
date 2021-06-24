#pragma once
#include <Eigen/Dense>

class MPC
{
public:
    /**
     * @brief Current state of the vehicle.
     */
    struct State
    {
        double x;   //!< X position in reference to local map.
        double y;   //!< Y position in reference to local map.
        double yaw; //!< Orientation of the robot in reference to local map.
        double cte; //!< Cross track error (signed distance of the vehicle to reference trajectory).
        double oe;  //!< Orientation error (difference between desired and current orientations).
    };

    /**
     * @brief Current actuation of the vehicle.
     */
    struct Inputs
    {
        double linear;
        double angular;
    };

    /**
     * @brief Goal of the vehicle.
     */
    struct Reference {
        double x;
        double y;
    };

    typedef Inputs Control; //!< Output of the MPC.

    struct Constraints {
        double linearMin;
        double linearMax;
        double angularMin;
        double angularMax;
    };

private:
    const unsigned int numSamples; //!< Number of samples for prediction and control horizons.

public:
    /**
     * @brief Class constructor.
     * @param horizonDuration Prediction and control horizons duration in seconds.
     * @param horizonSampling Sampling time of horizonDuration in seconds.
     */
    MPC(double horizonDuration, double horizonSampling);

    /**
     * @brief Get new control values.
     * @param state Current state of the vehicle.
     * @param inputs Current actuation of the vehicle.
     * @param goal Current goal of the vehicle.
     * @return New actuation for the vehicle.
     */
    Control update(const State &state, const Inputs &inputs, const Reference &goal);
};