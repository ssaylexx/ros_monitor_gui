#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import math
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QPushButton, QGroupBox, QGridLayout, QTabWidget)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Matplotlib imports
try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
except ImportError:
    print("Matplotlib required!")
    sys.exit(1)

class MplCanvas(FigureCanvas):
    def __init__(self, parent):
        self.fig = Figure(figsize=(5,4), dpi=100)
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.path_x, self.path_y = [], []
        self.line, = self.axes.plot([], [], 'b-')
        self.axes.set_title("Trajectory")
        
    def update_plot(self, x, y):
        self.path_x.append(x)
        self.path_y.append(y)
        self.line.set_data(self.path_x, self.path_y)
        # ВАЖНО: используем draw() -> вызывает мерцание
        self.draw() 

class RobotMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Monitor - v1.6")
        self.resize(900, 600)
        
        app_font = QFont("DejaVu Sans", 10)
        QApplication.setFont(app_font)
        
        main = QVBoxLayout()
        title = QLabel("ROBOT MONITOR")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 24px; font-weight: bold; margin: 20px;")
        main.addWidget(title)
        
        self.tabs = QTabWidget()
        dash = QWidget()
        ctrl_tab = QWidget()
        logs_tab = QWidget()
        self.tabs.addTab(dash, "Dashboard")
        self.tabs.addTab(ctrl_tab, "Control")
        self.tabs.addTab(logs_tab, "Logs")
        main.addWidget(self.tabs)
        
        # DASHBOARD
        dash_layout = QHBoxLayout(dash)
        left_col = QVBoxLayout()
        
        status = QGroupBox("Robot Status")
        sl = QVBoxLayout()
        self.pos_lbl = QLabel("Position: x=0.00 y=0.00")
        self.spd_lbl = QLabel("Speed: 0.00 m/s")
        self.yaw_lbl = QLabel("Yaw: 0.0 deg")
        for l in [self.pos_lbl, self.spd_lbl, self.yaw_lbl]:
            sl.addWidget(l)
        status.setLayout(sl)
        left_col.addWidget(status)
        
        # Добавляем график справа
        self.canvas = MplCanvas(self)
        dash_layout.addLayout(left_col)
        dash_layout.addWidget(self.canvas)
        
        # CONTROL
        ctrl_layout = QVBoxLayout(ctrl_tab)
        ctrl = QGroupBox("Manual Control")
        grid = QGridLayout()
        self.btn_fwd = QPushButton("^ Forward")
        self.btn_back = QPushButton("v Backward")
        self.btn_left = QPushButton("< Left")
        self.btn_right = QPushButton("> Right")
        self.btn_stop = QPushButton("STOP")
        grid.addWidget(self.btn_fwd, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_back, 2, 1)
        ctrl.setLayout(grid)
        ctrl_layout.addWidget(ctrl)
        
        self.setLayout(main)
        
        rospy.init_node('robot_monitor_gui', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(100) # Быстрое обновление для демонстрации мерцания
        
        self.last_x = 0.0; self.last_y = 0.0; self.last_yaw = 0.0; self.last_spd = 0.0
        
        self.btn_fwd.clicked.connect(lambda: self.send_cmd(0.5, 0))
        self.btn_back.clicked.connect(lambda: self.send_cmd(-0.5, 0))
        self.btn_left.clicked.connect(lambda: self.send_cmd(0, 1.0))
        self.btn_right.clicked.connect(lambda: self.send_cmd(0, -1.0))
        self.btn_stop.clicked.connect(lambda: self.send_cmd(0, 0))

    def odom_cb(self, msg):
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.last_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        self.last_spd = msg.twist.twist.linear.x
        # Обновляем график каждый кадр
        self.canvas.update_plot(self.last_x, self.last_y)

    def update_labels(self):
        self.pos_lbl.setText(f"Position: x={self.last_x:.2f} y={self.last_y:.2f}")
        self.spd_lbl.setText(f"Speed: {self.last_spd:.2f} m/s")
        self.yaw_lbl.setText(f"Yaw: {math.degrees(self.last_yaw):.1f} deg")

    def send_cmd(self, lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = RobotMonitorGUI()
    win.show()
    sys.exit(app.exec_())