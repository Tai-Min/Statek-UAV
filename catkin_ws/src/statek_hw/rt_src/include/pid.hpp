#pragma once

class PIDController
{
private:
    float previousErr = 0;
    float integral = 0;
    float previousControllVal = 0;

    float kp = 0;
    float ki = 0;
    float kd = 0;

public:
    float dt = 0;
    const float lowerSaturation;
    const float upperSaturation;

    PIDController(float ls = -1 * INFINITY, float us = INFINITY) : lowerSaturation(ls), upperSaturation(us) {}

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

    void setKp(float newKp) { kp = newKp; }
    void setKi(float newKi) { ki = newKi; }
    void setKd(float newKd) { kd = newKd; }
    void setSamplingTime(float _dt) { dt = _dt; }
};