#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import math
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QPushButton, QGroupBox, QGridLayout, QTabWidget, 
                           QTextEdit, QSlider)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
except ImportError:
    sys.exit(1)

class MplCanvas(FigureCanvas):
    def __init__(self, parent):
        self.fig = Figure(figsize=(5,4), dpi=100)
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.path_x, self.path_y = [], []
        self.line, = self.axes.plot([], [], 'b-', linewidth=2)
        self.robot_point, = self.axes.plot([], [], 'ro', markersize=8)
        self.axes.set_title("Trajectory")
        self.axes.grid(True, linestyle='--', alpha=0.7)
        self.axes.set_aspect('equal')
        self.axes.set_xlim(-1, 1); self.axes.set_ylim(-1, 1)
    def update_plot(self, x, y):
        if self.path_x:
            dist = math.sqrt((x - self.path_x[-1])**2 + (y - self.path_y[-1])**2)
            if dist < 0.01: return
        self.path_x.append(x); self.path_y.append(y)
        self.line.set_data(self.path_x, self.path_y)
        self.robot_point.set_data([x], [y])
        if len(self.path_x) > 5:
            margin = 1.5
            cx = (min(self.path_x) + max(self.path_x)) / 2
            cy = (min(self.path_y) + max(self.path_y)) / 2
            dx = max(max(self.path_x) - min(self.path_x), 2.0)
            dy = max(max(self.path_y) - min(self.path_y), 2.0)
            self.axes.set_xlim(cx - dx/2 - 0.5, cx + dx/2 + 0.5)
            self.axes.set_ylim(cy - dy/2 - 0.5, cy + dy/2 + 0.5)
        self.draw_idle()

class RobotMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Monitor - v2.0")
        self.resize(1000, 650)
        app_font = QFont("DejaVu Sans", 10)
        QApplication.setFont(app_font)
        
        # Переменные скорости
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        main = QVBoxLayout()
        title = QLabel("ROBOT MONITOR")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 24px; font-weight: bold; margin: 20px;")
        main.addWidget(title)
        
        self.tabs = QTabWidget()
        dash = QWidget(); ctrl_tab = QWidget(); logs_tab = QWidget()
        self.tabs.addTab(dash, "Dashboard"); self.tabs.addTab(ctrl_tab, "Control"); self.tabs.addTab(logs_tab, "Logs")
        main.addWidget(self.tabs)
        
        # DASHBOARD
        dash_layout = QHBoxLayout(dash)
        left_col = QVBoxLayout()
        status = QGroupBox("Robot Status")
        sl = QVBoxLayout()
        self.pos_lbl = QLabel("Position: x=0.00 y=0.00")
        self.spd_lbl = QLabel("Speed: 0.00 m/s")
        self.yaw_lbl = QLabel("Yaw: 0.0 deg")
        for l in [self.pos_lbl, self.spd_lbl, self.yaw_lbl]: sl.addWidget(l)
        status.setLayout(sl); left_col.addWidget(status)
        cam_group = QGroupBox("Camera Feed")
        cl = QVBoxLayout()
        self.cam_lbl = QLabel("Waiting for camera...")
        self.cam_lbl.setMinimumSize(320, 240); self.cam_lbl.setAlignment(Qt.AlignCenter)
        self.cam_lbl.setStyleSheet("background: black; color: white; border: 2px solid #333;")
        cl.addWidget(self.cam_lbl); cam_group.setLayout(cl); left_col.addWidget(cam_group)
        self.canvas = MplCanvas(self)
        dash_layout.addLayout(left_col); dash_layout.addWidget(self.canvas)
        
        # CONTROL с ползунками
        ctrl_layout = QVBoxLayout(ctrl_tab)
        
        spd_group = QGroupBox("Speed Settings")
        spd_lay = QHBoxLayout()
        
        ll = QVBoxLayout()
        ll.addWidget(QLabel("Linear (m/s)"))
        self.lin_sl = QSlider(Qt.Horizontal); self.lin_sl.setRange(1,10); self.lin_sl.setValue(5)
        self.lin_lbl = QLabel("0.5")
        self.lin_sl.valueChanged.connect(lambda v: self.set_speed(v, 'lin'))
        ll.addWidget(self.lin_sl); ll.addWidget(self.lin_lbl, alignment=Qt.AlignCenter)
        
        al = QVBoxLayout()
        al.addWidget(QLabel("Angular (rad/s)"))
        self.ang_sl = QSlider(Qt.Horizontal); self.ang_sl.setRange(1,20); self.ang_sl.setValue(10)
        self.ang_lbl = QLabel("1.0")
        self.ang_sl.valueChanged.connect(lambda v: self.set_speed(v, 'ang'))
        al.addWidget(self.ang_sl); al.addWidget(self.ang_lbl, alignment=Qt.AlignCenter)
        
        spd_lay.addLayout(ll); spd_lay.addLayout(al)
        spd_group.setLayout(spd_lay); ctrl_layout.addWidget(spd_group)
        
        ctrl = QGroupBox("Manual Control")
        grid = QGridLayout()
        self.btn_fwd = QPushButton("^ Forward"); self.btn_back = QPushButton("v Backward")
        self.btn_left = QPushButton("< Left"); self.btn_right = QPushButton("> Right")
        self.btn_stop = QPushButton("STOP")
        grid.addWidget(self.btn_fwd, 0, 1); grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1); grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_back, 2, 1)
        ctrl.setLayout(grid); ctrl_layout.addWidget(ctrl)
        
        # LOGS
        logs_layout = QVBoxLayout(logs_tab)
        self.log_txt = QTextEdit(); self.log_txt.setReadOnly(True)
        self.log_txt.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: 'Consolas', monospace; font-size: 12px;")
        logs_layout.addWidget(self.log_txt)
        
        self.setLayout(main)
        rospy.init_node('robot_monitor_gui', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.bridge = CvBridge()
        self.cam_sub = rospy.Subscriber('/spcbot/camera/image_raw', Image, self.cam_cb)
        self.timer = QTimer(); self.timer.timeout.connect(self.update_labels); self.timer.start(100)
        self.last_x = 0.0; self.last_y = 0.0; self.last_yaw = 0.0; self.last_spd = 0.0
        
        self.btn_fwd.clicked.connect(lambda: self.send_cmd(self.linear_speed, 0))
        self.btn_back.clicked.connect(lambda: self.send_cmd(-self.linear_speed, 0))
        self.btn_left.clicked.connect(lambda: self.send_cmd(0, self.angular_speed))
        self.btn_right.clicked.connect(lambda: self.send_cmd(0, -self.angular_speed))
        self.btn_stop.clicked.connect(lambda: self.send_cmd(0, 0))
        
        self.log_msg("System initialized. Speed sliders active.")

    def set_speed(self, v, t):
        if t == 'lin':
            self.linear_speed = v / 10.0
            self.lin_lbl.setText(f"{self.linear_speed:.1f}")
        else:
            self.angular_speed = v / 10.0
            self.ang_lbl.setText(f"{self.angular_speed:.1f}")

    def log_msg(self, msg):
        ts = rospy.get_time()
        self.log_txt.append(f"[{ts:.2f}] {msg}")
        sb = self.log_txt.verticalScrollBar(); sb.setValue(sb.maximum())

    def cam_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, ch = cv_img.shape
            qt_img = QImage(cv_img.data, w, h, ch*w, QImage.Format_RGB888).rgbSwapped()
            self.cam_lbl.setPixmap(QPixmap.fromImage(qt_img).scaled(
                self.cam_lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except CvBridgeError as e:
            self.log_msg(f"CvBridge Error: {e}")

    def odom_cb(self, msg):
        self.last_x = msg.pose.pose.position.x; self.last_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.last_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        self.last_spd = msg.twist.twist.linear.x
        self.canvas.update_plot(self.last_x, self.last_y)

    def update_labels(self):
        self.pos_lbl.setText(f"Position: x={self.last_x:.2f} y={self.last_y:.2f}")
        self.spd_lbl.setText(f"Speed: {self.last_spd:.2f} m/s")
        self.yaw_lbl.setText(f"Yaw: {math.degrees(self.last_yaw):.1f} deg")

    def send_cmd(self, lin, ang):
        t = Twist(); t.linear.x = lin; t.angular.z = ang; self.cmd_pub.publish(t)
        self.log_msg(f"CMD: lin={lin:.2f}, ang={ang:.2f}")

if __name__ == '__main__':
    app = QApplication(sys.argv); win = RobotMonitorGUI(); win.show(); sys.exit(app.exec_())