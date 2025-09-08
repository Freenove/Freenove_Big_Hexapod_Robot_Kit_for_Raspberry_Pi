#!/usr/bin/env python3

"""
Hexapod Robot Control GUI for ROS2 Jazzy.

This PyQt5 application provides a graphical user interface to monitor and control
a hexapod robot. It communicates with the robot's ROS2 nodes to send commands
and receive telemetry data.

Features:
- Main window with connection status, battery level, and telemetry grid.
- Grid layout visually represents the hexapod's legs and central components.
- Live telemetry is received via a subscription to the /joint_states topic.
- Pop-up windows for detailed control of each leg, the camera, and the IMU.
- Sliders and text boxes for precise joint and servo control.
- Video stream display for color and depth cameras.
- ROS2 communication (services, topics) handled in a non-blocking background thread.
- Publishes JointState messages for visualization in RViz2.
"""

import sys
import threading
import rclpy
import cv2
import numpy as np
import math
from rclpy.node import Node
from functools import partial

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton, QFrame,
                             QVBoxLayout, QHBoxLayout, QGridLayout, QSlider, QLineEdit,
                             QListWidget, QAbstractItemView, QSizePolicy,
                             QSpacerItem)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot, QSize, QTimer
from PyQt5.QtGui import QImage, QPixmap, QColor, QFont, QPainter, QPen, QBrush, QDoubleValidator

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from robot_interfaces.srv import SetServo

SERVO_CHANNELS = {
    'camera': {'pan': 0, 'tilt': 1},
    'leg1': {'coxa': 15, 'femur': 14, 'tibia': 13},
    'leg2': {'coxa': 12, 'femur': 11, 'tibia': 10},
    'leg3': {'coxa': 9, 'femur': 8, 'tibia': 31},
    'leg4': {'coxa': 22, 'femur': 23, 'tibia': 27},
    'leg5': {'coxa': 19, 'femur': 20, 'tibia': 21},
    'leg6': {'coxa': 16, 'femur': 17, 'tibia': 18},
}

REVERSE_SERVO_MAP = {}
for component, joints in SERVO_CHANNELS.items():
    for joint, channel in joints.items():
        REVERSE_SERVO_MAP[channel] = {'component': component, 'joint': joint}

RVIZ_JOINT_NAMES = {
    'leg1': {'coxa': 'leg1_coxa_joint', 'femur': 'leg1_femur_joint', 'tibia': 'leg1_tibia_joint'},
    'leg2': {'coxa': 'leg2_coxa_joint', 'femur': 'leg2_femur_joint', 'tibia': 'leg2_tibia_joint'},
    'leg3': {'coxa': 'leg3_coxa_joint', 'femur': 'leg3_femur_joint', 'tibia': 'leg3_tibia_joint'},
    'leg4': {'coxa': 'leg4_coxa_joint', 'femur': 'leg4_femur_joint', 'tibia': 'leg4_tibia_joint'},
    'leg5': {'coxa': 'leg5_coxa_joint', 'femur': 'leg5_femur_joint', 'tibia': 'leg5_tibia_joint'},
    'leg6': {'coxa': 'leg6_coxa_joint', 'femur': 'leg6_femur_joint', 'tibia': 'leg6_tibia_joint'},
    'camera': {'pan': 'camera_pan_joint', 'tilt': 'camera_tilt_joint'},
}

JOINT_NAME_TO_CHANNEL_MAP = {}
for component, joints in RVIZ_JOINT_NAMES.items():
    for joint_type, joint_name in joints.items():
        channel = SERVO_CHANNELS[component][joint_type]
        JOINT_NAME_TO_CHANNEL_MAP[joint_name] = channel

ROS_TOPICS = {
    'color': '/camera/camera/color/image_raw',
    'depth': '/camera/camera/depth/image_rect_raw'
}
ROS_SERVICES = {
    'set_servo': '/set_servo_angle'
}

class RosNodeThread(QThread):
    """ Manages all ROS2 communications in a background thread """
    color_image_signal = pyqtSignal(np.ndarray)
    depth_image_signal = pyqtSignal(np.ndarray)
    telemetry_update_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.node = None
        self.bridge = CvBridge()
        self.color_sub = None
        self.depth_sub = None

    def run(self):
        rclpy.init()
        self.node = rclpy.create_node('hexapod_gui_node')
        self.set_servo_client = self.node.create_client(SetServo, ROS_SERVICES['set_servo'])
        self.joint_state_pub = self.node.create_publisher(JointState, 'joint_states', 10)
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            'joint_states',
            self._joint_state_callback,
            10)

        self.node.get_logger().info("Hexapod GUI ROS2 Node is running.")
        rclpy.spin(self.node)

        self.node.destroy_node()
        rclpy.shutdown()

    def _joint_state_callback(self, msg):
        """Processes incoming JointState messages and converts them to the GUI's format."""
        telemetry_data = {}
        for joint_name, position_rad in zip(msg.name, msg.position):
            if joint_name in JOINT_NAME_TO_CHANNEL_MAP:
                channel = JOINT_NAME_TO_CHANNEL_MAP[joint_name]
                angle_deg = math.degrees(position_rad) + 90.0
                telemetry_data[channel] = angle_deg
        
        if telemetry_data:
            self.telemetry_update_signal.emit(telemetry_data)

    def _color_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_image_signal.emit(cv_image)

    def _depth_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image_signal.emit(cv_image)

    def subscribe_to_video(self, stream_type):
        if stream_type == 'color' and self.color_sub is None:
            self.color_sub = self.node.create_subscription(
                Image, ROS_TOPICS['color'], self._color_callback, 10)
            self.node.get_logger().info(f"Subscribed to {ROS_TOPICS['color']}")
        elif stream_type == 'depth' and self.depth_sub is None:
            self.depth_sub = self.node.create_subscription(
                Image, ROS_TOPICS['depth'], self._depth_callback, 10)
            self.node.get_logger().info(f"Subscribed to {ROS_TOPICS['depth']}")

    def unsubscribe_from_video(self, stream_type):
        if stream_type == 'color' and self.color_sub:
            self.node.destroy_subscription(self.color_sub)
            self.color_sub = None
            self.node.get_logger().info(f"Unsubscribed from {ROS_TOPICS['color']}")
        elif stream_type == 'depth' and self.depth_sub:
            self.node.destroy_subscription(self.depth_sub)
            self.depth_sub = None
            self.node.get_logger().info(f"Unsubscribed from {ROS_TOPICS['depth']}")

    def call_set_servo(self, channel, angle):
        if not self.set_servo_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(f"Service '{ROS_SERVICES['set_servo']}' not available.")
            return

        req = SetServo.Request()
        req.channel = int(channel)
        req.angle = int(angle)
        self.set_servo_client.call_async(req)
        self.node.get_logger().info(f"Set servo channel {channel} to {angle} degrees.")

    def publish_joint_state(self, joint_name, position_rad):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [joint_name]
        msg.position = [position_rad]
        self.joint_state_pub.publish(msg)

    def stop(self):
        if self.node and rclpy.ok():
            if rclpy.ok():
                rclpy.get_global_executor().shutdown()
                rclpy.shutdown()
        self.wait()

class JoystickWidget(QWidget):
    """ A simple visual placeholder for a joystick. """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(100, 100)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        size = min(self.width(), self.height())
        painter.setPen(QPen(QColor(120, 120, 120), 2))
        painter.drawEllipse(self.rect().center(), size // 2 - 5, size // 2 - 5)
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        painter.drawEllipse(self.rect().center(), size // 4, size // 4)

class HexapodImageCell(QLabel):
    """ A QLabel that draws a segment of the hexapod body. """
    def __init__(self, segment_type, parent=None):
        super().__init__(parent)
        self.segment_type = segment_type
        self.setMinimumSize(40, 40)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        pen = QPen(QColor(100, 120, 140), 6, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(pen)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        if self.segment_type == 'center_body':
            painter.setBrush(QBrush(QColor(100, 120, 140)))
            painter.drawEllipse(cx-15, cy-15, 30, 30)
        elif self.segment_type == 'front_right': painter.drawLine(0, h, cx, cy)
        elif self.segment_type == 'front_left':  painter.drawLine(w, h, cx, cy)
        elif self.segment_type == 'mid_right': painter.drawLine(0, cy, w, cy)
        elif self.segment_type == 'mid_left': painter.drawLine(w, cy, 0, cy)
        elif self.segment_type == 'rear_right': painter.drawLine(0, 0, cx, cy)
        elif self.segment_type == 'rear_left': painter.drawLine(w, 0, cx, cy)

class LegDisplayWindow(QWidget):

    def __init__(self, leg_name, ros_node_thread, parent=None):
        super().__init__(parent)
        self.leg_name = leg_name
        self.ros_node = ros_node_thread
        self.channels = SERVO_CHANNELS[leg_name.lower().replace(" ", "")]
        self.rviz_joints = RVIZ_JOINT_NAMES[leg_name.lower().replace(" ", "")]
        self.setWindowTitle(f"{leg_name} Control")
        self.setMinimumWidth(400)
        self.main_layout = QVBoxLayout(self)
        joint_frame = QFrame()
        joint_frame.setFrameShape(QFrame.StyledPanel)
        joint_layout = QGridLayout()
        joint_frame.setLayout(joint_layout)
        joint_layout.addWidget(QLabel(f"<b>Joint Control ({leg_name})</b>"), 0, 0, 1, 3)
        self.controls = {}
        joints = ['coxa', 'femur', 'tibia']
        for i, joint in enumerate(joints):
            self.controls[joint] = self._create_control_triplet(
                joint.capitalize(), 0, 180,
                partial(self._send_joint_command, joint, self.channels[joint])
            )
            joint_layout.addWidget(self.controls[joint]['label'], i + 1, 0)
            joint_layout.addWidget(self.controls[joint]['slider'], i + 1, 1)
            joint_layout.addWidget(self.controls[joint]['textbox'], i + 1, 2)
        ik_frame = QFrame()
        ik_frame.setFrameShape(QFrame.StyledPanel)
        ik_layout = QGridLayout()
        ik_frame.setLayout(ik_layout)
        ik_layout.addWidget(QLabel("<b>Inverse Kinematics (IK) Control</b>"), 0, 0, 1, 3)
        ik_params = ['X', 'Y', 'Z', 'Pitch', 'Yaw', 'Roll']
        for i, param in enumerate(ik_params):
            handler = lambda val, p=param: print(f"IK {p} set to {val}")
            ik_control = self._create_control_triplet(param, 0, 100, handler)
            ik_layout.addWidget(ik_control['label'], i + 1, 0)
            ik_layout.addWidget(ik_control['slider'], i + 1, 1)
            ik_layout.addWidget(ik_control['textbox'], i + 1, 2)
        self.main_layout.addWidget(joint_frame)
        self.main_layout.addWidget(ik_frame)

    def _create_control_triplet(self, name, min_val, max_val, handler_func):
        label = QLabel(name)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue((min_val + max_val) // 2)
        textbox = QLineEdit(str(float(slider.value())))
        textbox.setValidator(QDoubleValidator(min_val, max_val, 2))
        textbox.setFixedWidth(50)
        slider.valueChanged.connect(lambda val, tb=textbox: tb.setText(f"{val:.1f}"))
        textbox.textChanged.connect(lambda txt, sl=slider: sl.setValue(int(float(txt)) if txt else 0))
        slider.sliderReleased.connect(lambda sl=slider: handler_func(sl.value()))
        textbox.editingFinished.connect(lambda tb=textbox, sl=slider: handler_func(sl.value()))
        return {'label': label, 'slider': slider, 'textbox': textbox}
    
    def _send_joint_command(self, joint_name, channel, angle):
        print(f"COMMAND: Leg={self.leg_name}, Joint={joint_name}, Channel={channel}, Angle={angle}")
        self.ros_node.call_set_servo(channel, angle)
        angle_rad = np.deg2rad(angle - 90)
        rviz_joint_name = self.rviz_joints.get(joint_name)
        if rviz_joint_name:
            self.ros_node.publish_joint_state(rviz_joint_name, angle_rad)

class CameraDisplayWindow(QWidget):

    def __init__(self, ros_node_thread, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node_thread
        self.setWindowTitle("Camera Control & Display")
        self.setMinimumSize(1300, 600)
        self.main_layout = QVBoxLayout(self)
        control_frame = QFrame()
        control_frame.setFrameShape(QFrame.StyledPanel)
        control_layout = QGridLayout(control_frame)
        control_layout.addWidget(QLabel("<b>Camera Servo Control</b>"), 0, 0, 1, 3)
        pan_handler = partial(self._send_camera_command, 'pan', SERVO_CHANNELS['camera']['pan'])
        pan_controls = self._create_control_triplet("Pan", 0, 180, pan_handler)
        control_layout.addWidget(pan_controls['label'], 1, 0)
        control_layout.addWidget(pan_controls['slider'], 1, 1)
        control_layout.addWidget(pan_controls['textbox'], 1, 2)
        tilt_handler = partial(self._send_camera_command, 'tilt', SERVO_CHANNELS['camera']['tilt'])
        tilt_controls = self._create_control_triplet("Tilt", 0, 180, tilt_handler)
        control_layout.addWidget(tilt_controls['label'], 2, 0)
        control_layout.addWidget(tilt_controls['slider'], 2, 1)
        control_layout.addWidget(tilt_controls['textbox'], 2, 2)
        video_frame = QFrame()
        video_frame.setFrameShape(QFrame.StyledPanel)
        video_layout = QHBoxLayout(video_frame)
        color_vbox = QVBoxLayout()
        self.color_view = QLabel("Color Stream Off")
        self.color_view.setFixedSize(640, 480)
        self.color_view.setStyleSheet("background-color: black; color: white;")
        self.color_view.setAlignment(Qt.AlignCenter)
        self.color_toggle_btn = QPushButton("Enable Color Stream")
        self.color_toggle_btn.setCheckable(True)
        self.color_toggle_btn.toggled.connect(partial(self._toggle_video_stream, 'color'))
        color_vbox.addWidget(self.color_view)
        color_vbox.addWidget(self.color_toggle_btn)
        depth_vbox = QVBoxLayout()
        self.depth_view = QLabel("Depth Stream Off")
        self.depth_view.setFixedSize(640, 480)
        self.depth_view.setStyleSheet("background-color: black; color: white;")
        self.depth_view.setAlignment(Qt.AlignCenter)
        self.depth_toggle_btn = QPushButton("Enable Depth Stream")
        self.depth_toggle_btn.setCheckable(True)
        self.depth_toggle_btn.toggled.connect(partial(self._toggle_video_stream, 'depth'))
        depth_vbox.addWidget(self.depth_view)
        depth_vbox.addWidget(self.depth_toggle_btn)
        video_layout.addLayout(color_vbox)
        video_layout.addLayout(depth_vbox)
        self.main_layout.addWidget(control_frame)
        self.main_layout.addWidget(video_frame)
        self.ros_node.color_image_signal.connect(self._update_color_image)
        self.ros_node.depth_image_signal.connect(self._update_depth_image)

    def _create_control_triplet(self, name, min_val, max_val, handler_func):
        label = QLabel(name)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue((min_val + max_val) // 2)
        textbox = QLineEdit(str(float(slider.value())))
        textbox.setValidator(QDoubleValidator(min_val, max_val, 2))
        textbox.setFixedWidth(50)
        slider.valueChanged.connect(lambda val, tb=textbox: tb.setText(f"{val:.1f}"))
        textbox.textChanged.connect(lambda txt, sl=slider: sl.setValue(int(float(txt)) if txt else 0))
        slider.sliderReleased.connect(lambda sl=slider: handler_func(sl.value()))
        textbox.editingFinished.connect(lambda tb=textbox, sl=slider: handler_func(sl.value()))
        return {'label': label, 'slider': slider, 'textbox': textbox}
    
    def _send_camera_command(self, servo_name, channel, angle):
        print(f"COMMAND: Camera={servo_name}, Channel={channel}, Angle={angle}")
        self.ros_node.call_set_servo(channel, angle)
        angle_rad = np.deg2rad(angle - 90)
        rviz_joint_name = RVIZ_JOINT_NAMES['camera'].get(servo_name)
        if rviz_joint_name:
            self.ros_node.publish_joint_state(rviz_joint_name, angle_rad)

    def _toggle_video_stream(self, stream_type, checked):
        btn = self.color_toggle_btn if stream_type == 'color' else self.depth_toggle_btn
        if checked:
            self.ros_node.subscribe_to_video(stream_type)
            btn.setText(f"Disable {stream_type.capitalize()} Stream")
            btn.setStyleSheet("background-color: #ff6b6b;")
        else:
            self.ros_node.unsubscribe_from_video(stream_type)
            btn.setText(f"Enable {stream_type.capitalize()} Stream")
            btn.setStyleSheet("")
            view = self.color_view if stream_type == 'color' else self.depth_view
            view.setText(f"{stream_type.capitalize()} Stream Off")
            view.setStyleSheet("background-color: black; color: white;")

    @pyqtSlot(np.ndarray)
    def _update_color_image(self, cv_img):
        qt_img = self._convert_cv_qt(cv_img, 640, 480)
        self.color_view.setPixmap(qt_img)

    @pyqtSlot(np.ndarray)
    def _update_depth_image(self, cv_img):
        cv_img_normalized = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv_img_colormap = cv2.applyColorMap(cv_img_normalized, cv2.COLORMAP_JET)
        qt_img = self._convert_cv_qt(cv_img_colormap, 640, 480)
        self.depth_view.setPixmap(qt_img)

    def _convert_cv_qt(self, cv_img, width, height):
        if len(cv_img.shape) == 3:
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            convert_to_Qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)
        else:
             h, w = cv_img.shape
             bytes_per_line = w
             convert_to_Qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_Grayscale8)
        p = convert_to_Qt_format.scaled(width, height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
    
    def closeEvent(self, event):
        if self.color_toggle_btn.isChecked(): self.color_toggle_btn.toggle()
        if self.depth_toggle_btn.isChecked(): self.depth_toggle_btn.toggle()
        super().closeEvent(event)

class IMUDisplayWindow(QWidget):

    def __init__(self, ros_node_thread, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node_thread
        self.setWindowTitle("IMU Display")
        self.setMinimumWidth(300)
        layout = QVBoxLayout(self)
        reset_btn = QPushButton("Reset / Calibrate IMU")
        reset_btn.clicked.connect(self._send_imu_reset_command)
        layout.addWidget(reset_btn)
        self.value_labels = {}
        grid = QGridLayout()
        params = ['x', 'y', 'z', 'pitch', 'yaw', 'roll']
        for i, param in enumerate(params):
            grid.addWidget(QLabel(f"{param.capitalize()}:"), i, 0)
            self.value_labels[param] = QLabel("0.00")
            grid.addWidget(self.value_labels[param], i, 1)
        layout.addLayout(grid)
        self._update_imu_values({'x':0,'y':0,'z':0,'pitch':0,'yaw':0,'roll':0})

    def _send_imu_reset_command(self):
        print("COMMAND: Reset IMU to all zeros.")
        self._update_imu_values({'x':0,'y':0,'z':0,'pitch':0,'yaw':0,'roll':0})

    def _update_imu_values(self, imu_data):
        for key, value in imu_data.items():
            if key in self.value_labels:
                self.value_labels[key].setText(f"{value:.2f}")


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Hexapod Command Node")
        self.setGeometry(100, 100, 1000, 800)
        self.setStyleSheet("QFrame { border: 1px solid #aaa; border-radius: 5px; }")
        self.ros_thread = RosNodeThread()
        self.ros_thread.start()
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        self.secondary_windows = {}
        self.main_layout.addWidget(self._create_top_frame())
        self.main_layout.addWidget(self._create_middle_frame(), stretch=1)
        self.main_layout.addWidget(self._create_bottom_frame())
        self.ros_thread.telemetry_update_signal.connect(self._update_telemetry_display)
        self._update_connection_status(True)
        self._update_battery_level(88)

    def _create_top_frame(self):
        frame = QFrame()
        frame.setFixedHeight(40)
        layout = QHBoxLayout(frame)
        self.conn_status_label = QLabel("Status:")
        self.conn_status_value = QLabel("UNKNOWN")
        font = self.conn_status_value.font(); font.setBold(True)
        self.conn_status_value.setFont(font)
        layout.addWidget(self.conn_status_label)
        layout.addWidget(self.conn_status_value)
        layout.addStretch()
        self.batt_level_label = QLabel("Battery:")
        self.batt_level_value = QLabel("N/A")
        self.batt_level_value.setFont(font)
        layout.addWidget(self.batt_level_label)
        layout.addWidget(self.batt_level_value)
        return frame

    def _create_middle_frame(self):
        frame = QFrame()
        self.grid_layout = QGridLayout(frame)
        self.grid_layout.setSpacing(10)
        self.grid_layout.setRowStretch(0, 1)
        self.grid_layout.setRowStretch(6, 1)
        self.leg_value_labels = {}
        for i in range(1, 7):
            self.leg_value_labels[f'leg{i}'] = self._create_label_value_widget(['coxa', 'femur', 'tibia'])
        self.cam_value_labels = self._create_label_value_widget(['pan', 'tilt', 'Color', 'Depth'])
        self.imu_value_labels = self._create_label_value_widget(['X', 'Y', 'Z', 'Pitch', 'Yaw', 'Roll'])
        self.grid_layout.addWidget(self.leg_value_labels['leg6'], 1, 0, Qt.AlignCenter)
        self.grid_layout.addWidget(self._create_control_button("Leg 6", 6), 1, 1)
        self.grid_layout.addWidget(self.cam_value_labels, 1, 3, Qt.AlignCenter)
        self.grid_layout.addWidget(self._create_control_button("Leg 1", 1), 1, 5)
        self.grid_layout.addWidget(self.leg_value_labels['leg1'], 1, 6, Qt.AlignCenter)
        self.grid_layout.addWidget(HexapodImageCell('front_left'), 2, 2)
        self.grid_layout.addWidget(self._create_control_button("Camera"), 2, 3)
        self.grid_layout.addWidget(HexapodImageCell('front_right'), 2, 4)
        self.grid_layout.addWidget(self.leg_value_labels['leg5'], 3, 0, Qt.AlignCenter)
        self.grid_layout.addWidget(self._create_control_button("Leg 5", 5), 3, 1)
        self.grid_layout.addWidget(HexapodImageCell('mid_left'), 3, 2)
        self.grid_layout.addWidget(HexapodImageCell('center_body'), 3, 3)
        self.grid_layout.addWidget(HexapodImageCell('mid_right'), 3, 4)
        self.grid_layout.addWidget(self._create_control_button("Leg 2", 2), 3, 5)
        self.grid_layout.addWidget(self.leg_value_labels['leg2'], 3, 6, Qt.AlignCenter)
        self.grid_layout.addWidget(HexapodImageCell('rear_left'), 4, 2)
        self.grid_layout.addWidget(self._create_control_button("IMU"), 4, 3)
        self.grid_layout.addWidget(HexapodImageCell('rear_right'), 4, 4)
        self.grid_layout.addWidget(self.leg_value_labels['leg4'], 5, 0, Qt.AlignCenter)
        self.grid_layout.addWidget(self._create_control_button("Leg 4", 4), 5, 1)
        self.grid_layout.addWidget(self.imu_value_labels, 5, 3, Qt.AlignCenter)
        self.grid_layout.addWidget(self._create_control_button("Leg 3", 3), 5, 5)
        self.grid_layout.addWidget(self.leg_value_labels['leg3'], 5, 6, Qt.AlignCenter)
        return frame

    def _create_bottom_frame(self):
        frame = QFrame()
        frame.setFixedHeight(200)
        layout = QHBoxLayout(frame)
        trans_vbox = QVBoxLayout()
        trans_vbox.addWidget(QLabel("<b>Translational</b>"), alignment=Qt.AlignCenter)
        trans_vbox.addWidget(JoystickWidget())
        layout.addLayout(trans_vbox)
        rot_vbox = QVBoxLayout()
        rot_vbox.addWidget(QLabel("<b>Rotational</b>"), alignment=Qt.AlignCenter)
        rot_vbox.addWidget(JoystickWidget())
        layout.addLayout(rot_vbox)
        gait_vbox = QVBoxLayout()
        gait_vbox.addWidget(QLabel("<b>GAIT</b>"), alignment=Qt.AlignCenter)
        gait_list = QListWidget()
        gait_list.setSelectionMode(QAbstractItemView.MultiSelection)
        gait_list.addItems(['Tripod', 'Wave', 'Ripple', 'Tetrapod'])
        gait_vbox.addWidget(gait_list)
        layout.addLayout(gait_vbox)
        ai_vbox = QVBoxLayout()
        ai_vbox.addWidget(QLabel("<b>AI Mode</b>"), alignment=Qt.AlignCenter)
        ai_list = QListWidget()
        ai_list.setSelectionMode(QAbstractItemView.MultiSelection)
        ai_list.addItems(['Follow', 'Explore', 'Guard', 'Idle'])
        ai_vbox.addWidget(ai_list)
        layout.addLayout(ai_vbox)
        return frame

    def _create_label_value_widget(self, labels):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0,0,0,0)
        value_labels = {}
        for label_text in labels:
            h_layout = QHBoxLayout()
            h_layout.addWidget(QLabel(f"{label_text.capitalize()}:"))
            value_label = QLabel("N/A")
            value_label.setAlignment(Qt.AlignRight)
            value_labels[label_text.lower()] = value_label
            h_layout.addWidget(value_label)
            layout.addLayout(h_layout)
        widget.value_labels = value_labels
        return widget

    def _create_control_button(self, name, leg_num=None):
        button = QPushButton(name)
        button.setMinimumHeight(40)
        if "Leg" in name and leg_num is not None:
            button.clicked.connect(lambda: self._open_leg_display_handler(leg_num))
        elif "Camera" in name:
            button.clicked.connect(self._open_camera_display_handler)
        elif "IMU" in name:
            button.clicked.connect(self._open_imu_display_handler)
        return button

    @pyqtSlot(dict)
    def _update_telemetry_display(self, telemetry_data):
        """Receives telemetry data and updates the GUI labels."""
        for channel, angle in telemetry_data.items():
            if channel in REVERSE_SERVO_MAP:
                info = REVERSE_SERVO_MAP[channel]
                component = info['component']
                joint = info['joint']
                if angle == -1:
                    display_text = "Error"
                else:
                    display_text = f"{angle:.1f}°"
                if component.startswith('leg'):
                    self.leg_value_labels[component].value_labels[joint].setText(display_text)
                elif component == 'camera':
                    self.cam_value_labels.value_labels[joint].setText(display_text)

    def _update_connection_status(self, connected):
        if connected:
            self.conn_status_value.setText("Connected")
            self.conn_status_value.setStyleSheet("color: #4CAF50;")
        else:
            self.conn_status_value.setText("Disconnected")
            self.conn_status_value.setStyleSheet("color: #F44336;")

    def _update_battery_level(self, level):
        self.batt_level_value.setText(f"{level}%")
        color = "#4CAF50" if level > 50 else ("#FFC107" if level > 20 else "#F44336")
        self.batt_level_value.setStyleSheet(f"color: {color};")

    def _open_leg_display_handler(self, leg_number):
        win_id = f"leg_{leg_number}"
        if win_id not in self.secondary_windows or not self.secondary_windows[win_id].isVisible():
            leg_name = f"Leg {leg_number}"
            self.secondary_windows[win_id] = LegDisplayWindow(leg_name, self.ros_thread)
            self.secondary_windows[win_id].show()
        else:
            self.secondary_windows[win_id].activateWindow()

    def _open_camera_display_handler(self):
        win_id = "camera"
        if win_id not in self.secondary_windows or not self.secondary_windows[win_id].isVisible():
            self.secondary_windows[win_id] = CameraDisplayWindow(self.ros_thread)
            self.secondary_windows[win_id].show()
        else:
            self.secondary_windows[win_id].activateWindow()

    def _open_imu_display_handler(self):
        win_id = "imu"
        if win_id not in self.secondary_windows or not self.secondary_windows[win_id].isVisible():
            self.secondary_windows[win_id] = IMUDisplayWindow(self.ros_thread)
            self.secondary_windows[win_id].show()
        else:
            self.secondary_windows[win_id].activateWindow()

    def closeEvent(self, event):
        print("Closing application...")
        for window in self.secondary_windows.values():
            window.close()
        self.ros_thread.stop()
        event.accept()

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()