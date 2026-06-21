#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QPushButton, QGroupBox, QGridLayout)
from PyQt5.QtCore import Qt
from geometry_msgs.msg import Twist

class RobotMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Monitor - v1.1")
        self.resize(600, 500)
        
        main = QVBoxLayout()
        title = QLabel("ROBOT MONITOR")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 24px; font-weight: bold; margin: 20px;")
        main.addWidget(title)
        
        # Группа кнопок
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
        
        # Подключения
        self.btn_fwd.clicked.connect(lambda: self.send_cmd(0.5, 0))
        self.btn_back.clicked.connect(lambda: self.send_cmd(-0.5, 0))
        self.btn_left.clicked.connect(lambda: self.send_cmd(0, 1.0))
        self.btn_right.clicked.connect(lambda: self.send_cmd(0, -1.0))
        self.btn_stop.clicked.connect(lambda: self.send_cmd(0, 0))

    def send_cmd(self, lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)
        print(f"CMD: lin={lin}, ang={ang}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = RobotMonitorGUI()
    win.show()
    sys.exit(app.exec_())