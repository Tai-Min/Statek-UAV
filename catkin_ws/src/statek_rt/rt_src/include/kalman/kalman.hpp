#pragma once

#include <cstdint>

template <uint8_t numInputs, uint8_t numOutputs, uint8_t numStates>
class KalmanFilter
{
private:
    double stateMatrix[numStates * numStates];
    double inputMatrix[numInputs * numStates];
    double outputMatrix[numOutputs * numStates];

    // prediction
    double statePredictions[numStates];
    double covariancePredictions[numStates * numStates];

    // update
    double kalmanGain[numStates * numStates];
    double preFitResidual[numStates];
public:
    KalmanFilter();

    
};