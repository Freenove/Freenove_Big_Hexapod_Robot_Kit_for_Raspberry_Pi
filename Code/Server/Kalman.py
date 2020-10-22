import time
class Kalman_filter:
    def __init__(self,Q,R):
        self.Q = Q
        self.R = R
        self.P_k_k1 = 1
        self.Kg = 0
        self.P_k1_k1 = 1
        self.x_k_k1 = 0
        self.ADC_OLD_Value = 0
        self.Z_k = 0
        self.kalman_adc_old=0
        
    def kalman(self,ADC_Value):
        self.Z_k = ADC_Value
        if (abs(self.kalman_adc_old-ADC_Value)>=60):
            self.x_k1_k1= ADC_Value*0.400 + self.kalman_adc_old*0.600
        else:
            self.x_k1_k1 = self.kalman_adc_old;
        self.x_k_k1 = self.x_k1_k1
        self.P_k_k1 = self.P_k1_k1 + self.Q
        self.Kg = self.P_k_k1/(self.P_k_k1 + self.R)
        kalman_adc = self.x_k_k1 + self.Kg * (self.Z_k - self.kalman_adc_old)
        self.P_k1_k1 = (1 - self.Kg)*self.P_k_k1
        self.P_k_k1 = self.P_k1_k1
        self.kalman_adc_old = kalman_adc
        return kalman_adc
