# -*- coding: utf-8 -*-
import io
import time
import fcntl
import socket
import struct
from threading import Condition
import threading
from led import Led
from servo import Servo
from Thread import stop_thread
from buzzer import Buzzer
from control import Control
from adc import ADC
from ultrasonic import Ultrasonic
from command import COMMAND as cmd
from camera import Camera  

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

class Server:
    def __init__(self):
        # Initialize server state and components
        self.is_tcp_active = False
        self.is_servo_relaxed = False
        self.led_controller = Led()
        self.adc_sensor = ADC()
        self.servo_controller = Servo()
        self.buzzer_controller = Buzzer()
        self.control_system = Control()
        self.ultrasonic_sensor = Ultrasonic()
        self.camera_device = Camera()  
        self.led_thread = None 
        self.ultrasonic_thread = None  
        self.control_system.condition_thread.start()

    def get_interface_ip(self):
        # Get the IP address of the wlan0 interface
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(sock.fileno(),
                                            0x8915,
                                            struct.pack('256s', b'wlan0'[:15])
                                            )[20:24])

    def start_server(self):
        # Start the video and command servers
        host_ip = self.get_interface_ip()
        self.video_socket = socket.socket()
        self.video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.video_socket.bind((host_ip, 8002))
        self.video_socket.listen(1)
        self.command_socket = socket.socket()
        self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.command_socket.bind((host_ip, 5002))
        self.command_socket.listen(1)
        print('Server address: ' + host_ip)

    def stop_server(self):
        # Stop the video and command servers
        try:
            self.video_connection.close()
            self.command_connection.close()
        except:
            print('\n' + "No client connection")

    def reset_server(self):
        # Reset the server by stopping and then starting it again
        self.stop_server()
        self.start_server()
        self.video_thread = threading.Thread(target=self.transmit_video)
        self.command_thread = threading.Thread(target=self.receive_commands)
        self.video_thread.start()
        self.command_thread.start()

    def send_data(self, connection, data):
        # Send data over the specified connection
        try:
            connection.send(data.encode('utf-8'))
            # print("send",data)
        except Exception as e:
            print(e)

    def transmit_video(self):
        # Transmit video frames to the connected client
        try:
            self.video_connection, self.video_client_address = self.video_socket.accept()
            self.video_connection = self.video_connection.makefile('wb')
        except:
            pass
        self.video_socket.close()
        print("Video socket connected ... ")

        self.camera_device.start_stream()
        while True:
            try:
                frame = self.camera_device.get_frame() 
                frame_length = len(frame)
                # print("output .length:",lenFrame)
                length_binary = struct.pack('<I', frame_length)
                self.video_connection.write(length_binary)
                self.video_connection.write(frame)
            except Exception as e:
                self.camera_device.stop_stream()
                print("End transmit ... ")
                break

    def receive_commands(self):
        # Receive and process commands from the connected client
        try:
            self.command_connection, self.command_client_address = self.command_socket.accept()
            print("Client connection successful !")
        except:
            print("Client connect failed")
        self.command_socket.close()

        while True:
            try:
                received_data = self.command_connection.recv(1024).decode('utf-8')
            except:
                if self.is_tcp_active:
                    self.reset_server()
                    break
                else:
                    break
            if received_data == "" and self.is_tcp_active:
                self.reset_server()
                break
            else:
                command_array = received_data.split('\n')
                print(command_array)
                if command_array[-1] != "":
                    command_array = command_array[:-1]  
            for single_command in command_array:
                command_parts = single_command.split("#")
                if command_parts == None or command_parts[0] == '':
                    continue
                elif cmd.CMD_BUZZER in command_parts:
                    self.buzzer_controller.set_state(command_parts[1] == "1")
                elif cmd.CMD_POWER in command_parts:
                    try:
                        battery_voltage = self.adc_sensor.read_battery_voltage()
                        response_command = cmd.CMD_POWER + "#" + str(battery_voltage[0]) + "#" + str(battery_voltage[1]) + "\n"
                        # print(command)
                        self.send_data(self.command_connection, response_command)
                        if battery_voltage[0] < 5.5 or battery_voltage[1] < 6:
                            for _ in range(3):
                                self.buzzer_controller.set_state(True)
                                time.sleep(0.15)
                                self.buzzer_controller.set_state(False)
                                time.sleep(0.1)
                    except:
                        pass
                elif cmd.CMD_LED in command_parts:
                    try:
                        if self.led_thread is not None:
                            stop_thread(self.led_thread)
                    except:
                        pass
                    self.led_thread = threading.Thread(target=self.led_controller.process_light_command, args=(command_parts,))
                    self.led_thread.start()

                elif cmd.CMD_LED_MOD in command_parts:
                    try:
                        if self.led_thread is not None:
                            stop_thread(self.led_thread)
                    except:
                        pass
                    self.led_thread = threading.Thread(target=self.led_controller.process_light_command, args=(command_parts,))
                    self.led_thread.start()
                elif cmd.CMD_SONIC in command_parts:
                    response_command = cmd.CMD_SONIC + "#" + str(self.ultrasonic_sensor.get_distance()) + "\n"
                    self.send_data(self.command_connection, response_command)
                elif cmd.CMD_HEAD in command_parts:
                    if len(command_parts) == 3:
                        self.servo_controller.set_servo_angle(int(command_parts[1]), int(command_parts[2]))
                elif cmd.CMD_CAMERA in command_parts:
                    if len(command_parts) == 3:
                        x = self.control_system.restrict_value(int(command_parts[1]), 50, 180)
                        y = self.control_system.restrict_value(int(command_parts[2]), 0, 180)
                        self.servo_controller.set_servo_angle(0, x)
                        self.servo_controller.set_servo_angle(1, y)
                elif cmd.CMD_RELAX in command_parts:
                    if self.is_servo_relaxed == False:
                        self.control_system.relax(True)
                        self.is_servo_relaxed = True
                        print("relax")
                    else:
                        self.control_system.relax(False)
                        self.is_servo_relaxed = False
                        print("unrelax")
                elif cmd.CMD_SERVOPOWER in command_parts:
                    if command_parts[1] == "0":
                        self.control_system.servo_power_disable.on()
                    else:
                        self.control_system.servo_power_disable.off()

                else:
                    self.control_system.command_queue = command_parts
                    self.control_system.timeout = time.time()
        try:
            if self.led_thread is not None:
                stop_thread(self.led_thread)
        except:
            pass
        try:
            if self.ultrasonic_thread is not None:
                stop_thread(self.ultrasonic_thread)
        except:
            pass
        print("close_recv")

if __name__ == '__main__':
    pass