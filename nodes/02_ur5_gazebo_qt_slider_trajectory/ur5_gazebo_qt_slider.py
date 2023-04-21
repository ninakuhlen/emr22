#!/usr/bin/env python3
# --- ur5_gazebo_qt_slider.py ------
# Qt-Gui with Slider to move UR5 in Gazebo
# Version vom 21.4.2023 by OJ
# mit Ausgabe der Joiint-Position
# ----------------------------
# usage:
# $ roslaunch emr22 ur5_gazebo_bringup.launch
# ==== Gazebo Variante von Dairal, ohne MoveIt! ===
# $ rosrun emr22 ur5_gazebo_qt_slider.py
# ----------------------------

import sys
import rospy
import os

# Qt -------------------------------
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget, QLCDNumber, QSlider,
                             QPushButton, QVBoxLayout,
                             QHBoxLayout, QApplication,
                             QLabel)
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

# Name der Datei zum Speichern der Positionen
filename = "pose_ur5.txt"


class UIClass(QWidget):
    JointPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def cbGetJointPos1(self, rx_data):
        self.JointPos[3] = rx_data.process_value
        # print(self.JointPos)
    
    def cbGetJointPos2(self, rx_data):
        self.JointPos[4] = rx_data.process_value
        # print(self.JointPos)

    def __init__(self):  # Konstrukor
        # Konstruktor der Elternklasse aufrufen
        super(UIClass, self).__init__()
        self.initUI()

        self.wrist1_msg = Float64()
        self.wrist2_msg = Float64()
        rospy.init_node('ur5_gazebo_qt_slider', anonymous=True)

        self.pos_wrist1_pub = rospy.Publisher(
                              '/wrist_1_joint_position_controller/command',
                              Float64, queue_size=10)
        self.pos_wrist2_pub = rospy.Publisher(
                              '/wrist_2_joint_position_controller/command',
                              Float64, queue_size=10)

        self.rate = rospy.Rate(10)

        self.path = [[0.0, 0.0]]  # Initial Koordinaten

        # Joint Positions des GazeboBot holen
        rospy.Subscriber('/wrist_1_joint_position_controller/state',
                         JointControllerState, self.cbGetJointPos1)
        rospy.Subscriber('/wrist_2_joint_position_controller/state',
                         JointControllerState, self.cbGetJointPos2)

    def initUI(self):    # GUI - Instanziierung der Widgets
        self.lblInfo1 = QLabel('Wrist 1  * 10')
        LCDstartWert = 0

        # --- Wrist 1---
        self.lcd1 = QLCDNumber(self)
        self.lcd1.display(LCDstartWert)
        self.sld1 = QSlider(Qt.Horizontal, self)
        self.sld1.setMaximum(30)
        self.sld1.setMinimum(-30)
        self.sld1.setValue(LCDstartWert)
        self.pbLess1 = QPushButton('<')
        self.pbMore1 = QPushButton('>')

        self.lblInfo2 = QLabel('Wrist 2  * 10')
        LCDstartWert2 = 0

        # --- Wrist 2---
        self.lcd2 = QLCDNumber(self)
        self.lcd2.display(LCDstartWert2)
        self.sld2 = QSlider(Qt.Horizontal, self)
        self.sld2.setMaximum(30)
        self.sld2.setMinimum(-30)
        self.sld2.setValue(LCDstartWert2)
        self.pbLess2 = QPushButton('<')
        self.pbMore2 = QPushButton('>')

        # --- Buttons ---
        self.pbGo = QPushButton(' Go Home ')
        self.pbStore = QPushButton(' Store Pose ')
        self.pbRead = QPushButton(' Read  Pose ')

        self.lblStatus = QLabel('Status - Ausgabe')

        # BOX-Layout mit Widgets füllen
        vbox = QVBoxLayout()
        # ---- Wrist1 -----
        #  0.Reihe - Label
        hbox = QHBoxLayout()
        hbox.addWidget(self.lblInfo1)
        vbox.addLayout(hbox)
        # 1.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.lcd1)
        vbox.addLayout(hbox)
        # 2.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.sld1)
        vbox.addLayout(hbox)
        # 3.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.pbLess1)
        hbox.addWidget(self.pbMore1)
        vbox.addLayout(hbox)

        # ---- Wrist2 -----
        #  0.Reihe - Label
        hbox = QHBoxLayout()
        hbox.addWidget(self.lblInfo2)
        vbox.addLayout(hbox)
        # 1.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.lcd2)
        vbox.addLayout(hbox)
        # 2.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.sld2)
        vbox.addLayout(hbox)
        # 3.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.pbLess2)
        hbox.addWidget(self.pbMore2)
        vbox.addLayout(hbox)

        # Alle Boxen ins Window setzen
        self.setLayout(vbox)

        # Fenster Konfigurieren
        self.setGeometry(300, 300, 500, 150)
        self.setWindowTitle('EMR22 - UR5 - Gazebo Steering')
        self.show()

        # Signal und Slot verbinden
        self.sld1.valueChanged.connect(self.lcd1.display)
        self.sld1.valueChanged.connect(self.SlotPublish)  # publish to ROS
        self.pbLess1.clicked.connect(self.SlotKlick1)
        self.pbMore1.clicked.connect(self.SlotKlick1)

        self.sld2.valueChanged.connect(self.lcd2.display)
        self.sld2.valueChanged.connect(self.SlotPublish)  # publish to ROS
        self.pbLess2.clicked.connect(self.SlotKlick2)
        self.pbMore2.clicked.connect(self.SlotKlick2)

    def SlotKlick1(self):  # Wrist1 - Button < oder > gepusht
        sender = self.sender()
        self.lblStatus.setText(' X ' + sender.text() + ' was pressed')
        if sender.text() == '<':
            wert = self.sld1.value()
            wert = wert-1
            self.sld1.setValue(wert)
        else:
            wert = self.sld1.value()
            wert = wert+1
            self.sld1.setValue(wert)

    def SlotKlick2(self):  # Wrist2 - Button < oder > gepusht
        sender = self.sender()
        # self.lblStatus2.setText(' X ' + sender.text() + ' was pressed')
        if sender.text() == '<':
            wert = self.sld2.value()
            wert = wert-1
            self.sld2.setValue(wert)
        else:
            wert = self.sld2.value()
            wert = wert+1
            self.sld2.setValue(wert)

    def SlotGoHome(self):
        self.lblStatus.setText(' Go Home Button klicked ')
        self.sld1.setValue(0)
        # self.sld2.setValue(0)

    def SlotPublish(self):
        self.lblStatus.setText(' all topics publisht ')
        
        self.wrist1_msg = self.sld1.value()
        self.pos_wrist1_pub.publish(float(self.wrist1_msg)/10.0)
        
        self.wrist2_msg = self.sld2.value()
        self.pos_wrist2_pub.publish(float(self.wrist2_msg)/10.0)

        print(self.JointPos)

    def SlotStorePosition(self):
        # Get absolute Path
        self.lblStatus.setText(' Pose stored to file')
        myDirPath = os.path.dirname(os.path.abspath(__file__))
        print(myDirPath)
        myFilePath = os.path.join(myDirPath, filename)
        print(myFilePath)
        fobj = open(myFilePath, 'w')

        write_str = "[" + str(self.sld1.value()) + ","\
                    + str(0)\
                    + "] \n"
        fobj.write(write_str)
        fobj.close()
 
    def SlotReadPosition(self):
        # Get absolute Path
        self.lblStatus.setText(' Pose read from file ')
        myDirPath = os.path.dirname(os.path.abspath(__file__))
        print(myDirPath)
        myFilePath = os.path.join(myDirPath, filename)
        print(myFilePath)

        # Den vorgegebenen Pfad einlesen, jede Zeile ein Goal
        with open(myFilePath, 'r') as fin:
            for line in fin:
                self.path.append(eval(line))  # Goal anhaengen
        del self.path[0]  # [0, 0] entfernen (erstes Element )
        rospy.loginfo(str(self.path))
        # setze Slider mit den Werten aus der Datei
        self.sld1.setValue(self.path[0][0])
        # self.sld2.setValue(self.path[0][1])


if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        ui = UIClass()
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass
