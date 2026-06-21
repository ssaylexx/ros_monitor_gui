#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt

class RobotMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Monitor - v1.0")
        self.resize(600, 400)
        
        main = QVBoxLayout()
        title = QLabel("ROBOT MONITOR - v1.0")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 24px; font-weight: bold; margin: 20px;")
        main.addWidget(title)
        
        info = QLabel("ROS Node: robot_monitor_gui\nStatus: Initializing...")
        info.setAlignment(Qt.AlignCenter)
        main.addWidget(info)
        
        self.setLayout(main)
        
        # Инициализация ROS
        rospy.init_node('robot_monitor_gui', anonymous=True)
        self.log_message("System started.")

    def log_message(self, msg):
        print(f"[LOG] {msg}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = RobotMonitorGUI()
    win.show()
    sys.exit(app.exec_())