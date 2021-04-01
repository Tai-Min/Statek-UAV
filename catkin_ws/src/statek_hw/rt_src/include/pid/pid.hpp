#pragma once

class PIDController
{
private:
    double previousErr = 0;
    double integral = 0;
    double previousControllVal = 0;

    double kp = 0;
    double ki = 0;
    double kd = 0;

public:
    double dt = 1; //!< Delta time.
    const double lowerSaturation = INFINITY; //!< Maximum value that integrator may have.
    const double upperSaturation = -1 * INFINITY; //!< Minimum value that integrator may have.

    /**
    * @brief Class constructor.
    * 
    * @param ls Lower saturation value.
    * @param us Upper saturation value.
    */
    PIDController(double ls = -1 * INFINITY, double us = INFINITY) : lowerSaturation(ls), upperSaturation(us){}

    /**
     * @brief Receive control signal based on error value
     * 
     * @param err Error value.
     * @return Control value.
     */
    double read(double err);

    void setKp(double newKp) { kp = newKp; }
    void setKi(double newKi) { ki = newKi; }
    void setKd(double newKd) { kd = newKd; }
    void setSamplingTime(double _dt){ dt = dt;}
};

double PIDController::read(double err)
{
        //basic pid computing
        integral = integral + err * dt;
        double derivative = (err - previousErr) / dt;
        double controllVal = kp * err + ki * integral + kd * derivative;

        //saturation and integral clamping
        if (controllVal > upperSaturation)
        {
            //this if statement is experimental
            //apparently this metod (if I implemented it properly) is proven to be the best
            //method of integral clamping
            //but still it needs some testing
            if ((err > 0 && controllVal > 0) || (err < 0 && controllVal < 0))
                integral = integral - err * (dt / (double)1000); //revert changes to integral made in this iteration

            controllVal = upperSaturation; //saturate output
        }
        else if (controllVal < lowerSaturation)
        {
            //same as above
            if ((err > 0 && controllVal > 0) || (err < 0 && controllVal < 0))
                integral = integral - err * (dt / (double)1000);

            controllVal = lowerSaturation;
        }

        //save current stuff for next iteration
        previousErr = err;
        previousControllVal = controllVal;

        return controllVal;
}