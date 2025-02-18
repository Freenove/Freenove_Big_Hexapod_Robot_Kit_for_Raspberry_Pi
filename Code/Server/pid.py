#coding:utf-8
import time

class Incremental_PID:
    ''' PID controller'''
    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.target_value = 0.0
        self.kp = P
        self.ki = I
        self.kd = D
        self.last_error = 0.0
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        self.i_saturation = 10.0
        self.output = 0.0

    def pid_calculate(self, feedback_val):
        error = self.target_value - feedback_val
        self.p_error = self.kp * error
        self.i_error += error 
        self.d_error = self.kd * (error - self.last_error)
        if (self.i_error < -self.i_saturation):
            self.i_error = -self.i_saturation
        elif (self.i_error > self.i_saturation):
            self.i_error = self.i_saturation
        self.output = self.p_error + (self.ki * self.i_error) + self.d_error
        self.last_error = error
        return self.output

    def set_kp(self, proportional_gain):
        self.kp = proportional_gain

    def set_ki(self, integral_gain):
        self.ki = integral_gain

    def set_kd(self, derivative_gain):
        self.kd = derivative_gain

    def set_i_saturation(self, saturation_val):
        self.i_saturation = saturation_val

    def set_target_value(self, target):
        self.target_value = target