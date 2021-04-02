#pragma once

class LowPassFilter
{
private:
  float previousOutput = 0;

public:
  const float smoothingFactor = 1;

  LowPassFilter(float _sf = 1) : smoothingFactor(_sf){};

  float read(float input)
  {
    float output = previousOutput + smoothingFactor * (input - previousOutput);
    previousOutput = output;
    return output;
  }
};