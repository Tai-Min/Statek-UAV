#pragma once

class LowPassFilter
{
private:
  float previousOutput = 0;

public:
  const float smoothingFactor = 1;

  /**
   * @brief Class constructor.
   * @param _sf Smoothing factor (between 0 and 1).
   */
  LowPassFilter(float _sf = 1) : smoothingFactor(_sf){};

  /**
   * @brief Process input through low pass filter.
   * @param input Input to process.
   * @return Output of the low pass filter.
   */
  float read(float input)
  {
    float output = previousOutput + smoothingFactor * (input - previousOutput);
    previousOutput = output;
    return output;
  }
};