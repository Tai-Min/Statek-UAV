import numpy as np
class Kalman():
    def __init__(self, initial_position, Q, R):
        # @brief Class constructor.
        # @param initial_position Initial position (x, y).
        # @param Q Process variance.
        # @param R Measurement variance.

        # Parameters.
        self.Q = Q
        self.R = R

        # Initial conditions.
        x = initial_position[0]
        y = initial_position[1]

        self.a_posteriori_xhat = np.array([[x], [y]])
        self.a_posteriori_P = self.Q

    def update(self, position_measurement, velocity_input):
        # @brief Update filter.
        # @param position_measurement Current position measurement.
        # @param Time passed since last update.
        # @return Position estimate (x, y).

        z = np.array([[position_measurement[0]], [position_measurement[1]]])

        B = np.array([[0.5, 0], [0, 0.5]])
        u = np.array([[velocity_input[0]], [velocity_input[1]]])

        # Predict.
        a_priori_xhat = self.a_posteriori_xhat + B @ u
        a_priori_P = self.a_posteriori_P + self.Q

        # Update.
        innovation = z - a_priori_xhat
        innovation_covariance = a_priori_P + self.R
        K = a_priori_P @ np.transpose(innovation_covariance)
        self.a_posteriori_xhat = a_priori_xhat + K @ innovation
        self.a_posteriori_P = (np.identity(2) - K) @ a_priori_P

        x = self.a_posteriori_xhat[0]
        y = self.a_posteriori_xhat[1]

        return (x[0], y[0])