import sys
import threading
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
from robot_interfaces.srv import SetServo

# ROS 2 Node running in a separate thread
class RosNodeThread(QThread):
    image_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = rclpy.create_node('gui_ros_node')
        # self.bridge = CvBridge()
        # self.subscription = self.node.create_subscription(
        #     Image, '/camera/depth/image_rect_raw', self.image_callback, 10)
    
    # def image_callback(self, msg):
    #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #     self.image_signal.emit(cv_image)

    def run(self):
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

# Main PyQt5 Window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Panel")
        self.ros_thread = RosNodeThread()
        self.ros_node = self.ros_thread.node
        self.set_servo_client = self.ros_node.create_client(SetServo, 'set_servo')

        # --- UI Setup ---
        self.video_label = QLabel("Waiting for depth stream...")
        self.video_label.setFixedSize(640, 480)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-90, 90)
        self.slider.setValue(0)
        self.angle_label = QLabel(f"Angle: {self.slider.value()}°")

        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        layout.addWidget(self.angle_label)
        layout.addWidget(self.slider)
        
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # --- Connections ---
        # self.ros_thread.image_signal.connect(self.update_image)
        self.slider.valueChanged.connect(self.update_angle_label)
        self.slider.sliderReleased.connect(self.call_set_servo_service)
        
        # --- Start ROS Thread ---
        self.ros_thread.start()

    # @pyqtSlot(np.ndarray)
    # def update_image(self, cv_img):
    #     # Normalize and colorize the depth image
    #     cv_img = cv2.normalize(cv_img, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
    #     cv_img = cv2.applyColorMap(cv_img, cv2.COLORMAP_JET)

    #     # Convert to QPixmap
    #     h, w, ch = cv_img.shape
    #     bytes_per_line = ch * w
    #     convert_to_Qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
    #     p = QPixmap.fromImage(convert_to_Qt_format)
    #     self.video_label.setPixmap(p)

    def update_angle_label(self, value):
        self.angle_label.setText(f"Angle: {value}°")

    def call_set_servo_service(self):
        while not self.set_servo_client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().info('SetServo service not available, waiting again...')
        
        req = SetServo.Request()
        req.angle_degrees = float(self.slider.value())
        self.set_servo_client.call_async(req)
        self.ros_node.get_logger().info(f"Requested servo angle: {req.angle_degrees}°")

    def closeEvent(self, event):
        self.ros_thread.quit()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()