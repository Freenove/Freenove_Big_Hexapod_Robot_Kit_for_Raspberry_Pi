class Kalman_filter:
    def __init__(self, process_noise_covariance, measurement_noise_covariance):
        self.process_noise_covariance = process_noise_covariance  # Process noise covariance (Q)
        self.measurement_noise_covariance = measurement_noise_covariance  # Measurement noise covariance (R)
        self.estimated_error_covariance = 1  # Initial estimate error covariance (P_k_k1)
        self.kalman_gain = 0  # Kalman gain (Kg)
        self.posterior_error_covariance = 1  # Posterior estimate error covariance (P_k1_k1)
        self.posterior_estimate = 0  # Posterior estimate of state (x_k_k1)
        self.previous_adc_value = 0  # Previous ADC value
        self.current_measurement = 0  # Current measurement (Z_k)
        self.previous_kalman_output = 0  # Previous Kalman filter output

    def kalman(self, adc_value):
        self.current_measurement = adc_value
        # Handle large changes in ADC value
        if abs(self.previous_kalman_output - adc_value) >= 60:
            self.posterior_estimate = adc_value * 0.400 + self.previous_kalman_output * 0.600
        else:
            self.posterior_estimate = self.previous_kalman_output
        # Update estimate error covariance (P_k_k1 = P_k1_k1 + Q)
        self.estimated_error_covariance = self.posterior_error_covariance + self.process_noise_covariance
        # Calculate Kalman gain (Kg = P_k_k1 / (P_k_k1 + R))
        self.kalman_gain = self.estimated_error_covariance / (self.estimated_error_covariance + self.measurement_noise_covariance)
        # Calculate Kalman filter output (x_k_k1 = x_k1_k1 + Kg * (Z_k - x_k1_k1))
        kalman_output = self.posterior_estimate + self.kalman_gain * (self.current_measurement - self.previous_kalman_output)
        # Update posterior estimate error covariance (P_k1_k1 = (1 - Kg) * P_k_k1)
        self.posterior_error_covariance = (1 - self.kalman_gain) * self.estimated_error_covariance
        # Update previous Kalman filter output
        self.previous_kalman_output = kalman_output
        return kalman_output

if __name__ == '__main__':
    kalman_filter = Kalman_filter(0.001, 0.1)
    for i in range(100):
        kalman_output = kalman_filter.kalman(i)
        print(f"Value: {i}, Kalman Output: {kalman_output}")