#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import math
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QPushButton, QGroupBox, QGridLayout)
from PyQt5.QtCore import Qt, QTimer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Monitor - v1.3")
        self.resize(600, 600)
        
        main = QVBoxLayout()
        title = QLabel("ROBOT MONITOR")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 24px; font-weight: bold; margin: 20px;")
        main.addWidget(title)
        
        # Статус
        status = QGroupBox("Robot Status")
        sl = QVBoxLayout()
        self.pos_lbl = QLabel("Position: x=0.00 y=0.00")
        self.spd_lbl = QLabel("Speed: 0.00 m/s")
        self.yaw_lbl = QLabel("Yaw: 0.0 deg")
        for l in [self.pos_lbl, self.spd_lbl, self.yaw_lbl]:
            sl.addWidget(l)
        status.setLayout(sl)
        main.addWidget(status)
        
        # Кнопки
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
        main.addWidget(ctrl)
        
        self.setLayout(main)
        
        rospy.init_node('robot_monitor_gui', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        
        # Таймер обновления текста
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(400)
        
        # Данные
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.last_spd = 0.0
        
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