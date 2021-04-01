#pragma once

class LowPassFilter
{
private:
  double previousOutput = 0; //!< Object's memory.

public:
  const double smoothingFactor = 1;//!< Smoothing factor. 1 - everything passes through, 0 - nothing passes through.

  /**
   * @brief Class constructor
   * 
   * @param _sf Smoothing factor. 1 - everything passes through, 0 - nothing passes through.
   */
  LowPassFilter(double _sf = 1) : smoothingFactor(_sf){};


  /**
   * @brief Pass input signal through the filter.
   * 
   * @param input Input to the filter.
   * @return Filtered input.
   */
  double read(double input)
  {
    double output = previousOutput + smoothingFactor * (input - previousOutput);
    previousOutput = output;
    return output;
  }
};