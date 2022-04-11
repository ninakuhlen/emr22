#!/usr/bin/env python3
# --- ur5_gazebo_qt_slider.py ------
# Qt-Gui with Slider to move UR5 in Gazebo
# Version vom 30.3.2022 by OJ
# ----------------------------
# usage:
# $ roslaunch emr22 ur5_gazebo_pid.launch
# $ rosrun emr22 ur5_gazebo_qt_slider.py
# ----------------------------

import sys
import rospy

# Qt -------------------------------
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget, QLCDNumber, QSlider,
                             QPushButton, QVBoxLayout,
                             QHBoxLayout, QApplication,
                             QLabel)
from std_msgs.msg import Float64

# Vollstaendiger Pfad der Datei zum Speichern der Positionen, "~" funkt nicht
filename = "/home/oj/ws_moveit/src/emr22/nodes/ue01_ur5_gazebo_qt_slider/pose_ur5.txt"
######## oj gegen ihren Username ersetzen !!! ##################

class UIClass(QWidget):
    def __init__(self):  # Konstrukor
        # Konstruktor der Elternklasse aufrufen
        super(UIClass, self).__init__()
        self.initUI()

        self.wrist1_msg = Float64()
        self.wrist2_msg = Float64()
        rospy.init_node('ur5_gazebo_qt_slider', anonymous=True)

        self.pos_wrist1_pub = rospy.Publisher('/wrist_1_joint_position_controller/command',
                                              Float64, queue_size=10)
        self.pos_wrist2_pub = rospy.Publisher('/wrist_2_joint_position_controller/command',
                                              Float64, queue_size=10)
        self.rate = rospy.Rate(10)

        self.path = [[0.0, 0.0]]  # Initial Koordinaten

    def initUI(self):
        # Instanziierung der Widgets
        self.lblInfoX = QLabel('Wrist 1  * 10')
        LCDstartWert = 0
        self.lcdX = QLCDNumber(self)
        self.lcdX.display(LCDstartWert)
        
        self.sldX = QSlider(Qt.Horizontal, self)
        self.sldX.setMaximum(30)
        self.sldX.setMinimum(-30)
        self.sldX.setValue(LCDstartWert)

        self.pbLessX = QPushButton('<')
        self.pbMoreX = QPushButton('>')

        self.lblInfo2 = QLabel('Wrist 2  * 10')
        self.lcd2 = QLCDNumber(self)
        self.lcd2.display(LCDstartWert)
        
        self.sld2 = QSlider(Qt.Horizontal, self)
        self.sld2.setMaximum(30)
        self.sld2.setMinimum(-30)
        self.sld2.setValue(LCDstartWert)

        self.pbLess2 = QPushButton('<')
        self.pbMore2 = QPushButton('>')

        self.pbGo = QPushButton(' Go Home ')
        self.pbStore = QPushButton(' Store Pose ')
        self.pbRead = QPushButton(' Read  Pose ')
    
        self.lblStatus = QLabel('Status - Ausgabe')

        # BOX-Layout mit Widgets füllen
        vbox = QVBoxLayout()
        #  0.Reihe - Label
        hbox = QHBoxLayout()
        hbox.addWidget(self.lblInfoX)
        vbox.addLayout(hbox)
        # 1.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.lcdX)
        vbox.addLayout(hbox)
        # 2.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.sldX)
        vbox.addLayout(hbox)

        # 3.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.pbLessX)
        hbox.addWidget(self.pbMoreX)
        vbox.addLayout(hbox)


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

        # 4.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.pbGo)
        hbox.addWidget(self.pbStore)
        hbox.addWidget(self.pbRead)
        vbox.addLayout(hbox)

        # 5.Reihe
        hbox = QHBoxLayout()
        hbox.addWidget(self.lblStatus)
        vbox.addLayout(hbox)

        # Alle Boxen ins Window setzen
        self.setLayout(vbox)

        # Fenster Konfigurieren
        self.setGeometry(300, 300, 500, 150)
        self.setWindowTitle('EMR22 - UR5 - Gazebo Steering')
        self.show()

        # Signal und Slot verbinden
        self.sldX.valueChanged.connect(self.lcdX.display)
        self.sldX.valueChanged.connect(self.SlotPublish)  # publish to ROS
        self.pbLessX.clicked.connect(self.SlotKlickX)
        self.pbMoreX.clicked.connect(self.SlotKlickX)

        self.sld2.valueChanged.connect(self.lcd2.display)
        self.sld2.valueChanged.connect(self.SlotPublish)  # publish to ROS
        self.pbLess2.clicked.connect(self.SlotKlick2)
        self.pbMore2.clicked.connect(self.SlotKlick2)
        self.pbGo.clicked.connect(self.SlotGoHome)
        self.pbStore.clicked.connect(self.SlotStorePosition)
        self.pbRead.clicked.connect(self.SlotReadPosition)

    def SlotKlickX(self):
        sender = self.sender()
        self.lblStatus.setText(' X ' + sender.text() + ' was pressed')
        if sender.text() == '<':
            wert = self.sldX.value()
            wert = wert-1
            self.sldX.setValue(wert)
        else:
            wert = self.sldX.value()
            wert = wert+1
            self.sldX.setValue(wert)

    def SlotKlick2(self):
        sender = self.sender()
        self.lblStatus.setText(' X ' + sender.text() + ' was pressed')
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
        self.pos_wrist1_pub.publish(0.0)
        self.pos_wrist2_pub.publish(0.0)

    def SlotPublish(self):
        self.wrist1_msg = self.sldX.value()
        self.pos_wrist1_pub.publish(float(self.wrist1_msg)/10.0)
        self.wrist2_msg = self.sld2.value()
        self.pos_wrist2_pub.publish(float(self.wrist2_msg)/10.0)

    def SlotStorePosition(self):
        fobj = open(filename, 'w')
        write_str = "[" + str(self.sldX.value()) + ","\
                    + str(self.sld2.value())\
                    + "] \n"
        fobj.write(write_str)
        fobj.close()
 
    def SlotReadPosition(self):
        # Den vorgegebenen Pfad einlesen, jede Zeile ein Goal
        with open(filename, 'r') as fin:
            for line in fin:
                self.path.append(eval(line))  # Goal anhaengen
        del self.path[0]  # [0, 0] entfernen (ertes Element )
        rospy.loginfo(str(self.path))
        # setze Slider mit den Werten aus der Datei
        self.sldX.setValue(self.path[0][0])
        self.sld2.setValue(self.path[0][1])

    

if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        ui = UIClass()
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass
