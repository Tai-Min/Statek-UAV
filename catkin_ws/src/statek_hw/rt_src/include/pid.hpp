#pragma once

class PIDController
{
private:
    float previousErr = 0;         //!< For PID algorithm.
    float integral = 0;            //!< Integrator's value.
    float previousControllVal = 0; //!< For PID algorithm.

    float kp = 0; //!< Proportional gain.
    float ki = 0; //!< Integrator's gain.
    float kd = 0; //!< Derivative gain.

public:
    float dt = 0;                //!< Sampling time.
    const float lowerSaturation; //!< Saturation for negative overflow.
    const float upperSaturation; //!< Saturation for positive overflow.

    /**
     * @brief Class constructor.
     * @param ls Lower saturation.
     * @param us Upper saturation.
     */
    PIDController(float ls = -1 * INFINITY, float us = INFINITY) : lowerSaturation(ls), upperSaturation(us) {}

    /**
     * @brief Read control value.
     * @param err Control error.
     * @return Control value saturated or 0 if dt <= 0.
     */
    float read(float err)
    {
        // Something gone wrong.
        if (dt <= 0)
            return 0;

        // Basic pid computing.
        integral = integral + err * dt;
        float derivative = (err - previousErr) / dt;
        float controllVal = kp * err + ki * integral + kd * derivative;

        // Saturation and integral clamping.
        if (controllVal > upperSaturation)
        {
            // This if statement is experimental.
            // Apparently this metod (if I implemented it properly) is proven to be the best
            // method of integral clamping but still it needs some testing.
            if ((err > 0 && controllVal > 0) || (err < 0 && controllVal < 0))
                integral = integral - err * dt; // Revert changes to integral made in this iteration.

            controllVal = upperSaturation; // Saturate output.
        }
        else if (controllVal < lowerSaturation)
        {
            // Same as above.
            if ((err > 0 && controllVal > 0) || (err < 0 && controllVal < 0))
                integral = integral - err * dt;

            controllVal = lowerSaturation;
        }

        // Save current stuff for next iteration.
        previousErr = err;
        previousControllVal = controllVal;

        return controllVal;
    }

    /**
     * @brief Set kp.
     */
    void setKp(float newKp) { kp = newKp; }

    /**
     * @brief Set ki.
     */
    void setKi(float newKi) { ki = newKi; }

    /**
     * @brief Set kd.
     */
    void setKd(float newKd) { kd = newKd; }

    /**
     * @brief Set sampling time.
     */
    void setSamplingTime(float _dt) { dt = _dt; }
};