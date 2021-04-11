#pragma once

class LowPassFilter
{
private:
  float previousOutput = 0; //!< Object's memory.

public:
  float smoothingFactor = 1; //!< Smoothing factor between 0 and 1.

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
    float output = this->previousOutput + this->smoothingFactor * (input - this->previousOutput);
    this->previousOutput = output;
    return output;
  }

  /**
   * @brief Set smoothing factor.
   * @param _sf Smoothing factor to set.
   */
  void setSmoothingFactor(float _sf){
    if(_sf < 0)
      _sf = 0;
    if(_sf > 1)
      _sf = 1;
    this->smoothingFactor = _sf;
  }
};