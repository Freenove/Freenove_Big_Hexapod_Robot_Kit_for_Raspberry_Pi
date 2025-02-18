#coding:utf-8
import time
import math
import os
from kalman import Kalman_filter
from mpu6050 import mpu6050

class IMU:
    def __init__(self):
        self.proportional_gain = 100 
        self.integral_gain = 0.002 
        self.half_time_step = 0.001 

        self.quaternion_w = 1
        self.quaternion_x = 0
        self.quaternion_y = 0
        self.quaternion_z = 0

        self.integral_error_x = 0
        self.integral_error_y = 0
        self.integral_error_z = 0
        self.pitch_angle = 0
        self.roll_angle = 0
        self.yaw_angle = 0
    
        self.sensor = mpu6050(address=0x68, bus=1) 
        self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)   
        self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)  
    
        self.kalman_filter_AX = Kalman_filter(0.001, 0.1)
        self.kalman_filter_AY = Kalman_filter(0.001, 0.1)
        self.kalman_filter_AZ = Kalman_filter(0.001, 0.1)

        self.kalman_filter_GX = Kalman_filter(0.001, 0.1)
        self.kalman_filter_GY = Kalman_filter(0.001, 0.1)
        self.kalman_filter_GZ = Kalman_filter(0.001, 0.1)
    
        self.error_accel_data, self.error_gyro_data = self.calculate_average_sensor_data()
    
    def calculate_average_sensor_data(self):
        accel_x_sum = 0
        accel_y_sum = 0
        accel_z_sum = 0
        
        gyro_x_sum = 0
        gyro_y_sum = 0
        gyro_z_sum = 0
        
        for _ in range(100):
            accel_data = self.sensor.get_accel_data()   
            gyro_data = self.sensor.get_gyro_data()      
            
            accel_x_sum += accel_data['x']
            accel_y_sum += accel_data['y']
            accel_z_sum += accel_data['z']
            
            gyro_x_sum += gyro_data['x']
            gyro_y_sum += gyro_data['y']
            gyro_z_sum += gyro_data['z']
            
        accel_x_avg = accel_x_sum / 100
        accel_y_avg = accel_y_sum / 100
        accel_z_avg = accel_z_sum / 100
        
        gyro_x_avg = gyro_x_sum / 100
        gyro_y_avg = gyro_y_sum / 100
        gyro_z_avg = gyro_z_sum / 100
        
        accel_data['x'] = accel_x_avg
        accel_data['y'] = accel_y_avg
        accel_data['z'] = accel_z_avg - 9.8
        
        gyro_data['x'] = gyro_x_avg
        gyro_data['y'] = gyro_y_avg
        gyro_data['z'] = gyro_z_avg
        return accel_data, gyro_data

    def update_imu_state(self):
        accel_data = self.sensor.get_accel_data()    
        gyro_data = self.sensor.get_gyro_data() 
        
        accel_x = self.kalman_filter_AX.kalman(accel_data['x'] - self.error_accel_data['x'])
        accel_y = self.kalman_filter_AY.kalman(accel_data['y'] - self.error_accel_data['y'])
        accel_z = self.kalman_filter_AZ.kalman(accel_data['z'] - self.error_accel_data['z'])
        gyro_x = self.kalman_filter_GX.kalman(gyro_data['x'] - self.error_gyro_data['x'])
        gyro_y = self.kalman_filter_GY.kalman(gyro_data['y'] - self.error_gyro_data['y'])
        gyro_z = self.kalman_filter_GZ.kalman(gyro_data['z'] - self.error_gyro_data['z'])

        accel_norm = math.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z)
        
        accel_x /= accel_norm
        accel_y /= accel_norm
        accel_z /= accel_norm
        
        quat_vx = 2 * (self.quaternion_x * self.quaternion_z - self.quaternion_w * self.quaternion_y)
        quat_vy = 2 * (self.quaternion_w * self.quaternion_x + self.quaternion_y * self.quaternion_z)
        quat_vz = self.quaternion_w * self.quaternion_w - self.quaternion_x * self.quaternion_x - self.quaternion_y * self.quaternion_y + self.quaternion_z * self.quaternion_z
        
        error_x = (accel_y * quat_vz - accel_z * quat_vy)
        error_y = (accel_z * quat_vx - accel_x * quat_vz)
        error_z = (accel_x * quat_vy - accel_y * quat_vx)
        
        self.integral_error_x += error_x * self.integral_gain
        self.integral_error_y += error_y * self.integral_gain
        self.integral_error_z += error_z * self.integral_gain
        
        gyro_x += self.proportional_gain * error_x + self.integral_error_x
        gyro_y += self.proportional_gain * error_y + self.integral_error_y
        gyro_z += self.proportional_gain * error_z + self.integral_error_z
        
        self.quaternion_w += (-self.quaternion_x * gyro_x - self.quaternion_y * gyro_y - self.quaternion_z * gyro_z) * self.half_time_step
        self.quaternion_x += (self.quaternion_w * gyro_x + self.quaternion_y * gyro_z - self.quaternion_z * gyro_y) * self.half_time_step
        self.quaternion_y += (self.quaternion_w * gyro_y - self.quaternion_x * gyro_z + self.quaternion_z * gyro_x) * self.half_time_step
        self.quaternion_z += (self.quaternion_w * gyro_z + self.quaternion_x * gyro_y - self.quaternion_y * self.quaternion_x) * self.half_time_step
        
        norm = math.sqrt(self.quaternion_w * self.quaternion_w + self.quaternion_x * self.quaternion_x + self.quaternion_y * self.quaternion_y + self.quaternion_z * self.quaternion_z)
        self.quaternion_w /= norm
        self.quaternion_x /= norm
        self.quaternion_y /= norm
        self.quaternion_z /= norm
        
        current_pitch = math.asin(-2 * self.quaternion_x * self.quaternion_z + 2 * self.quaternion_w * self.quaternion_y) * 57.3
        current_roll = math.atan2(2 * self.quaternion_y * self.quaternion_z + 2 * self.quaternion_w * self.quaternion_x, -2 * self.quaternion_x * self.quaternion_x - 2 * self.quaternion_y * self.quaternion_y + 1) * 57.3
        current_yaw = math.atan2(2 * (self.quaternion_x * self.quaternion_y + self.quaternion_w * self.quaternion_z), self.quaternion_w * self.quaternion_w + self.quaternion_x * self.quaternion_x - self.quaternion_y * self.quaternion_y - self.quaternion_z * self.quaternion_z) * 57.3
        self.pitch_angle = current_pitch
        self.roll_angle = current_roll
        self.yaw_angle = current_yaw
        return self.pitch_angle, self.roll_angle, self.yaw_angle

    def handle_exception(self, exception):
        print(exception)
        os.system("i2cdetect -y 1")
        raise exception

if __name__ == '__main__':
    imu_sensor = IMU()
    start_time = time.time()
    while True:
        try:    
            time.sleep(0.01)
            roll_angle, pitch_angle, yaw_angle = imu_sensor.update_imu_state()
            print(roll_angle, pitch_angle, yaw_angle)
        except Exception as e:
            imu_sensor.handle_exception(e)
