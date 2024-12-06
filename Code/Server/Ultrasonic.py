import time
from gpiozero import DistanceSensor, PWMSoftwareFallback
import warnings
class Ultrasonic:
    def __init__(self):
        warnings.filterwarnings("ignore", category=DeprecationWarning)
        warnings.filterwarnings("ignore", category=PWMSoftwareFallback)  # Ignore PWM software fallback warnings
        trigger_pin = 27
        echo_pin    = 22
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin ,max_distance=3)

    def get_distance(self):     # get the measurement results of ultrasonic module,with unit: cm
        distance_cm = self.sensor.distance * 100
        return  int(distance_cm)
        
# Main program logic follows:
if __name__ == '__main__':
    pass
        
