#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import math
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QPushButton, QGroupBox, QGridLayout, 
                           QTabWidget, QTextEdit, QSlider)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap, QFont

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
except ImportError:
    print("Ошибка: Не установлен matplotlib. Выполните: pip install matplotlib")
    sys.exit(1)

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super(MplCanvas, self).__init__(self.fig)
        self.setParent(parent)
        
        self.axes.set_title('Robot Trajectory')
        self.axes.set_xlabel('X (m)')
        self.axes.set_ylabel('Y (m)')
        self.axes.grid(True, linestyle='--', alpha=0.7)
        self.axes.set_aspect('equal')
        
        self.path_x, self.path_y = [], []
        self.line, = self.axes.plot([], [], 'b-', linewidth=2, label='Path')
        self.robot_point, = self.axes.plot([], [], 'ro', markersize=8, label='Robot')
        self.axes.legend(loc='upper left')
        
        self.axes.set_xlim(-1, 1)
        self.axes.set_ylim(-1, 1)
        self.fig.tight_layout()

    def update_plot(self, x, y):
        self.path_x.append(x)
        self.path_y.append(y)
        self.line.set_data(self.path_x, self.path_y)
        self.robot_point.set_data([x], [y])
        
        margin = 1.5
        curr_xlim = self.axes.get_xlim()
        curr_ylim = self.axes.get_ylim()
        
        need_update = False
        if x > curr_xlim[1] - margin or x < curr_xlim[0] + margin:
            need_update = True
        if y > curr_ylim[1] - margin or y < curr_ylim[0] + margin:
            need_update = True
        if len(self.path_x) < 5:
            need_update = True

        if need_update:
            min_x, max_x = min(self.path_x), max(self.path_x)
            min_y, max_y = min(self.path_y), max(self.path_y)
            
            dx = max(max_x - min_x, 2.0) 
            dy = max(max_y - min_y, 2.0)
            
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            
            self.axes.set_xlim(center_x - dx/2 - 0.5, center_x + dx/2 + 0.5)
            self.axes.set_ylim(center_y - dy/2 - 0.5, center_y + dy/2 + 0.5)
        
        self.draw_idle()


class RobotMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROBOT MONITOR - v2.2")
        self.resize(1100, 750)

        app_font = QFont("DejaVu Sans", 10)
        QApplication.setFont(app_font)

        self.setStyleSheet("""
            QWidget { background-color: #f0f0f0; }
            QGroupBox { font-weight: bold; border: 1px solid #ccc; border-radius: 8px; margin-top: 1em; padding-top: 15px; }
            QGroupBox::title { subcontrol-origin: margin; left: 15px; padding: 0 8px; color: #00008B; }
            QPushButton { padding: 12px; font-size: 15px; border-radius: 6px; background-color: #e0e0e0; border: 1px solid #bbb; }
            QPushButton:hover { background-color: #d0d0d0; border-color: #999; }
            QPushButton:pressed { background-color: #0B61A4; color: white; border-color: #033E6B; }
            QLabel { padding: 6px; }
            QTextEdit { background-color: #1e1e1e; color: #00ff00; font-family: 'Consolas', monospace; font-size: 12px; border-radius: 4px; }
            QSlider::groove:horizontal { height: 8px; background: #ccc; border-radius: 4px; }
            QSlider::handle:horizontal { background: #0B61A4; width: 18px; margin: -5px 0; border-radius: 9px; }
        """)

        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.last_x, self.last_y, self.last_yaw = 0.0, 0.0, 0.0
        self.last_speed = 0.0
        self.data_received = False
        self.bridge = CvBridge()

        self.turn_timer = QTimer()
        self.turn_timer.setSingleShot(True)
        self.turn_timer.timeout.connect(self.stop_robot)

        main_layout = QVBoxLayout()
        
        title = QLabel("ROBOT MONITOR & CONTROL DASHBOARD")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 26px; font-weight: bold; color: #033E6B; padding: 15px;")
        main_layout.addWidget(title)

        self.tabs = QTabWidget()
        self.dashboard_tab = QWidget()
        self.control_tab = QWidget()
        self.logs_tab = QWidget()
        
        self.tabs.addTab(self.dashboard_tab, "Dashboard")
        self.tabs.addTab(self.control_tab, "Control")
        self.tabs.addTab(self.logs_tab, "Logs")
        
        main_layout.addWidget(self.tabs)
        self.setLayout(main_layout)
        
        self.setup_dashboard_tab()
        self.setup_control_tab()
        self.setup_logs_tab()

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.camera_sub = rospy.Subscriber('/spcbot/camera/image_raw', Image, self.image_callback) 

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_status_labels)
        self.update_timer.start(100)
        
        self.log_message("System initialized successfully.")

    def setup_dashboard_tab(self):
        layout = QHBoxLayout(self.dashboard_tab)
        
        left_col = QVBoxLayout()
        
        status_group = QGroupBox("Robot Status")
        status_layout = QVBoxLayout()
        self.pos_label = QLabel("Position: x=0.00  y=0.00")
        self.speed_label = QLabel("Speed: 0.00 m/s")
        self.yaw_label = QLabel("Yaw: 0.0 deg")
        self.status_label = QLabel("Status:  Waiting for /odom...")
        for lbl in [self.pos_label, self.speed_label, self.yaw_label, self.status_label]:
            status_layout.addWidget(lbl)
        status_group.setLayout(status_layout)
        left_col.addWidget(status_group)
        
        camera_group = QGroupBox("Camera Feed")
        camera_layout = QVBoxLayout()
        self.camera_label = QLabel("Waiting for camera feed...")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(320, 240)
        self.camera_label.setStyleSheet("background-color: black; color: white; border: 2px solid #333; border-radius: 4px;")
        camera_layout.addWidget(self.camera_label)
        camera_group.setLayout(camera_layout)
        left_col.addWidget(camera_group)
        
        layout.addLayout(left_col)
        
        path_group = QGroupBox("Trajectory")
        path_layout = QVBoxLayout()
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        path_layout.addWidget(self.canvas)
        path_group.setLayout(path_layout)
        layout.addWidget(path_group)

    def setup_control_tab(self):
        layout = QVBoxLayout(self.control_tab)
        
        speed_group = QGroupBox("Speed Settings")
        speed_layout = QHBoxLayout()
        
        lin_speed_layout = QVBoxLayout()
        lin_speed_layout.addWidget(QLabel("Linear Speed (m/s)"))
        self.lin_speed_slider = QSlider(Qt.Horizontal)
        self.lin_speed_slider.setRange(1, 10)
        self.lin_speed_slider.setValue(5)
        self.lin_speed_label = QLabel(f"{self.linear_speed:.1f}")
        self.lin_speed_slider.valueChanged.connect(lambda v: self.update_speed(v, 'linear'))
        lin_speed_layout.addWidget(self.lin_speed_slider)
        lin_speed_layout.addWidget(self.lin_speed_label, alignment=Qt.AlignCenter)
        
        ang_speed_layout = QVBoxLayout()
        ang_speed_layout.addWidget(QLabel("Angular Speed (rad/s)"))
        self.ang_speed_slider = QSlider(Qt.Horizontal)
        self.ang_speed_slider.setRange(1, 20)
        self.ang_speed_slider.setValue(10)
        self.ang_speed_label = QLabel(f"{self.angular_speed:.1f}")
        self.ang_speed_slider.valueChanged.connect(lambda v: self.update_speed(v, 'angular'))
        ang_speed_layout.addWidget(self.ang_speed_slider)
        ang_speed_layout.addWidget(self.ang_speed_label, alignment=Qt.AlignCenter)
        
        speed_layout.addLayout(lin_speed_layout)
        speed_layout.addLayout(ang_speed_layout)
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        ctrl_group = QGroupBox("Manual Control")
        grid = QGridLayout()

        self.btn_forward = QPushButton("^ Forward")
        self.btn_backward = QPushButton("v Backward")
        self.btn_left = QPushButton("< Left")
        self.btn_right = QPushButton("> Right")
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet("background-color: #ff4d4d; color: white; font-weight: bold; border: 1px solid #cc0000;")

        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_backward, 2, 1)
        
        self.btn_forward.clicked.connect(lambda: self.send_cmd(self.linear_speed, 0, "FORWARD"))
        self.btn_backward.clicked.connect(lambda: self.send_cmd(-self.linear_speed, 0, "BACKWARD"))
        self.btn_left.clicked.connect(lambda: self.send_cmd(0, self.angular_speed, "LEFT"))
        self.btn_right.clicked.connect(lambda: self.send_cmd(0, -self.angular_speed, "RIGHT"))
        self.btn_stop.clicked.connect(self.stop_robot)

        ctrl_group.setLayout(grid)
        layout.addWidget(ctrl_group)

        actions_group = QGroupBox("Special Maneuvers")
        actions_layout = QHBoxLayout()

        self.turn_90_btn = QPushButton("Turn 90 CW")
        self.turn_180_btn = QPushButton("Turn 180")
        self.reset_pos_btn = QPushButton("Reset Pos")
        
        self.turn_90_btn.clicked.connect(lambda: self.execute_turn(90))
        self.turn_180_btn.clicked.connect(lambda: self.execute_turn(180))
        self.reset_pos_btn.clicked.connect(self.reset_position)
        
        actions_layout.addWidget(self.turn_90_btn)
        actions_layout.addWidget(self.turn_180_btn)
        actions_layout.addWidget(self.reset_pos_btn)
        
        actions_group.setLayout(actions_layout)
        layout.addWidget(actions_group)
        
        layout.addStretch()

    def setup_logs_tab(self):
        layout = QVBoxLayout(self.logs_tab)
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        layout.addWidget(self.log_display)
    
    def log_message(self, message):
        timestamp = rospy.get_time()
        self.log_display.append(f"[{timestamp:.2f}] {message}")
        scrollbar = self.log_display.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def update_speed(self, value, type):
        if type == 'linear':
            self.linear_speed = value / 10.0
            self.lin_speed_label.setText(f"{self.linear_speed:.1f}")
        elif type == 'angular':
            self.angular_speed = value / 10.0
            self.ang_speed_label.setText(f"{self.angular_speed:.1f}")

    def send_cmd(self, lin, ang, name=""):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)
        if name:
            self.log_message(f"CMD: {name} (lin={lin:.2f}, ang={ang:.2f})")

    def stop_robot(self):
        self.send_cmd(0, 0, "STOP")
        if self.turn_timer.isActive():
            self.turn_timer.stop()

    def execute_turn(self, degrees):
        if self.turn_timer.isActive():
            self.turn_timer.stop()
            self.send_cmd(0, 0)
            
        self.log_message(f"Starting turn: {degrees} degrees")
        target_rad = math.radians(degrees)
        duration = abs(target_rad) / self.angular_speed
        
        direction = 1 if degrees > 0 else -1
        self.send_cmd(0, direction * self.angular_speed, f"TURN {degrees}")
        
        self.turn_timer.start(int(duration * 1000))
        self.log_message(f"Turn will complete in {duration:.1f}s")

    def reset_position(self):
        self.log_message("Resetting position to (0,0,0)...")
        
        try:
            pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = 0.0
            msg.pose.pose.position.y = 0.0
            msg.pose.pose.orientation.w = 1.0
            pose_pub.publish(msg)
            self.log_message("Published reset to /initialpose")
        except Exception as e:
            self.log_message(f"Warning: Could not publish reset: {e}")
            
        if hasattr(self, 'canvas'):
            self.canvas.path_x.clear()
            self.canvas.path_y.clear()
            self.canvas.line.set_data([], [])
            self.canvas.robot_point.set_data([], [])
            self.canvas.axes.set_xlim(-1, 1)
            self.canvas.axes.set_ylim(-1, 1)
            self.canvas.draw_idle()
            self.log_message("Trajectory plot cleared.")
        
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0

    def odom_cb(self, msg):
        self.data_received = True
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.last_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        self.last_speed = msg.twist.twist.linear.x
        
        if hasattr(self, 'canvas') and self.canvas.path_x:
            dist = math.sqrt((self.last_x - self.canvas.path_x[-1])**2 + 
                             (self.last_y - self.canvas.path_y[-1])**2)
            if dist > 0.01:
                self.canvas.update_plot(self.last_x, self.last_y)
        elif hasattr(self, 'canvas'):
            self.canvas.update_plot(self.last_x, self.last_y)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(qt_image)
            self.camera_label.setPixmap(pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except CvBridgeError as e:
            self.log_message(f"CvBridge Error: {e}")

    def update_status_labels(self):
        if self.data_received:
            self.pos_label.setText(f"Position: x={self.last_x:.2f}  y={self.last_y:.2f}")
            self.speed_label.setText(f"Speed: {self.last_speed:.2f} m/s")
            self.yaw_label.setText(f"Yaw: {math.degrees(self.last_yaw):.1f} deg")
            self.status_label.setText("Status: OK - Receiving data")
        else:
            self.status_label.setText("Status: Waiting for /odom topic...")

    def closeEvent(self, event):
        self.stop_robot()
        rospy.signal_shutdown("GUI closed")
        event.accept()


if __name__ == '__main__':
    rospy.init_node('robot_monitor_gui', anonymous=True)
    app = QApplication(sys.argv)
    win = RobotMonitorGUI()
    win.show()
    sys.exit(app.exec_())