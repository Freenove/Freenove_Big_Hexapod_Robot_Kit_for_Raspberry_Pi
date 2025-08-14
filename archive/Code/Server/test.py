import time

def test_Led():
    from led import Led
    led = Led()
    try:
        #Red wipe
        print ("\nRed wipe")
        led.color_wipe([255, 0, 0]) 
        time.sleep(1)

        #Green wipe
        print ("\nGreen wipe")
        led.color_wipe([0, 255, 0]) 
        time.sleep(1)
    
        #Blue wipe
        print ("\nBlue wipe")
        led.color_wipe([0, 0, 255]) 
        time.sleep(1)

        #White wipe
        print ("\nWhite wipe")
        led.color_wipe([255, 255, 255]) 
        time.sleep(1)
    
        led.color_wipe([0, 0, 0])   #turn off the light
        print ("\nEnd of program")
    except KeyboardInterrupt:
        led.color_wipe([0, 0, 0])   #turn off the light
        print ("\nEnd of program")

def test_Ultrasonic():
    from ultrasonic import Ultrasonic
    ultrasonic=Ultrasonic()
    try:
        while True:
            data=ultrasonic.get_distance()   #Get the value
            print ("Obstacle distance is "+str(data)+"CM")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print ("\nEnd of program")

def test_Servo():
    from servo import Servo
    servo = Servo()
    try:
        for i in range(50):
            servo.set_servo_angle(15,90+i)
            servo.set_servo_angle(12,90+i)
            servo.set_servo_angle(9,90+i)
            servo.set_servo_angle(16,90+i)
            servo.set_servo_angle(19,90+i)
            servo.set_servo_angle(22,90+i)
            time.sleep(0.005)
        for i in range(60):
            servo.set_servo_angle(14,90+i)
            servo.set_servo_angle(11,90+i)
            servo.set_servo_angle(8,90+i)
            servo.set_servo_angle(17,90-i)
            servo.set_servo_angle(20,90-i)
            servo.set_servo_angle(23,90-i)
            time.sleep(0.005)
        for i in range(120):
            servo.set_servo_angle(13,i)
            servo.set_servo_angle(10,i)
            servo.set_servo_angle(31,i)
            servo.set_servo_angle(18,180-i)
            servo.set_servo_angle(21,180-i)
            servo.set_servo_angle(27,180-i)
            time.sleep(0.005)
        print ("\nEnd of program")      
        servo.relax()
    except KeyboardInterrupt:
        servo.relax()
        print ("\nEnd of program")

def test_Adc():
    from adc import ADC
    adc = ADC()
    try:
        while True:
            Power=adc.read_battery_voltage()
            print ("The battery voltage is "+str(Power)+'\n')
            time.sleep(1)
    except KeyboardInterrupt:
        print ("\nEnd of program")

def test_Buzzer():
    from buzzer import Buzzer
    buzzer = Buzzer()
    try:
        buzzer.set_state(True)
        time.sleep(1)
        print ("1S")
        time.sleep(1)
        print ("2S")
        time.sleep(1)
        print ("3S")
        buzzer.set_state(False)
        print ("End of program")
    except KeyboardInterrupt:
        buzzer.set_state(False)
        print ("End of program")
        

if __name__ == '__main__':
    print ('Program is starting ... ')
    import sys
    if len(sys.argv)<2:
        print ("Parameter error: Please assign the device")
        exit() 
    if sys.argv[1] == 'Led':
        test_Led()
    elif sys.argv[1] == 'Ultrasonic':
        test_Ultrasonic()
    elif sys.argv[1] == 'Servo': 
        test_Servo()               
    elif sys.argv[1] == 'ADC':   
        test_Adc()  
    elif sys.argv[1] == 'Buzzer':   
        test_Buzzer()
        
        
        
        
