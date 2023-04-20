#!/usr/bin/env python3
# --- ue03_ur5_gazebo_qt_slider_trajectory.py ------
# Qt-Gui with Slider to move UR5 in Gazebo
# uses trajectory controller 
# like rqt 
# Version vom 20.4.2023 by OJ
# ----------------------------
# usage:
# $ rosrun emr22 starthilfe_ur5  => UR5 in Gazebo und MoveIt!
# $ rosrun emr22 ue03_ur5_gazebo_qt_slider_trajectory.py
# ----------------------------

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# for more explications
# see http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

import sys
import rospy
import os  # für  os.path.abspath(__file__)

# Qt -------------------------------
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget, QLCDNumber, QSlider,
                             QPushButton, QVBoxLayout,
                             QHBoxLayout, QApplication,
                             QLabel)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

# Vollstaendiger Pfad der Datei zum Speichern der Positionen
myFilePath = os.path.dirname(os.path.abspath(__file__))
print(myFilePath)
filename = os.path.join(myFilePath, "pose_ur5.txt")


class UIClass(QWidget):
    JointPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    def cbGetJointPos(self, rx_data):
        # rospy.loginfo(rospy.get_caller_id() + " I heard %s", rx_data.actual.positions)
        self.JointPos  = rx_data.actual.positions
        # print(self.JointPos)

    def __init__(self):  # Konstrukor
        # Konstruktor der Elternklasse aufrufen
        super(UIClass, self).__init__()
        self.initUI()

        self.wrist1_msg = JointTrajectory()
       
        rospy.init_node('ue03_ur5_gazebo_qt_slider_trajectory', anonymous=True)

        self.wrist1_pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command',
                                              JointTrajectory, queue_size=10)        
        # Joint Positions des RealBot holen
        rospy.Subscriber('/scaled_pos_joint_traj_controller/state' ,JointTrajectoryControllerState, self.cbGetJointPos)
        
        self.rate = rospy.Rate(10)
        self.path = [[0.0, 0.0]]  # Initial Koordinaten

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

        # --- Buttons ---
        self.pbGo = QPushButton(' set Slider ZeroPosition')
        self.pbStore = QPushButton(' Store Pose ')
        self.pbRead = QPushButton(' Go to stored Pose ')

        self.lblStatus = QLabel('Status - Ausgabe')

        # BOX-Layout mit Widgets fuellen
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
        self.sld1.valueChanged.connect(self.lcd1.display)
        self.sld1.valueChanged.connect(self.SlotPublish)  # publish to ROS
        self.pbLess1.clicked.connect(self.SlotKlick1)
        self.pbMore1.clicked.connect(self.SlotKlick1)

        self.pbGo.clicked.connect(self.SlotGoHome)
        self.pbStore.clicked.connect(self.SlotStorePosition)
        self.pbRead.clicked.connect(self.SlotReadPosition)

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

    def SlotGoHome(self):
        self.lblStatus.setText(' Go Home Button klicked ')
        self.sld1.setValue(0)
        self.SlotPublish()
 
    def SlotPublish(self):
        self.lblStatus.setText(' alle Topics werden gepublisht ')
        self.msg = JointTrajectory()
        self.jt_ur5 = JointTrajectory()
        self.jt_ur5.joint_names = ['elbow_joint', 'shoulder_lift_joint',
                                   'shoulder_pan_joint', 'wrist_1_joint',
                                   'wrist_2_joint', 'wrist_3_joint']

       
        # Erst schauen wo der UR5 steht
        print("Current Joint Positions")
        print(self.JointPos)       

        # hier die aktuellen Positionen nutzen, statt irgendwelcher Konstanten
        # rostopic info /scaled_pos_joint_traj_controller/state
        # ACHTUNG! [0] und [2] vertauscht
        jtpt = JointTrajectoryPoint()        
        jtpt.positions = [self.JointPos[2],  # elbow_joint
                          self.JointPos[1],  # shoulder_lift_joint
                          self.JointPos[0],  # shoulder_pan_joint
                          self.JointPos[3] + self.sld1.value()/10.0,
                          self.JointPos[4],
                          self.JointPos[5]]  
        
        jtpt.time_from_start = rospy.Duration.from_sec(2)
        # je länger desto langsamer
        # https://www.programcreek.com/python/example/123228/trajectory_msgs.msg.JointTrajectory
        # example 7
        # siehe auch http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

        self.jt_ur5.points.append(jtpt)
        self.wrist1_pub.publish(self.jt_ur5)

    def SlotStorePosition(self):
        self.lblStatus.setText(' Pose stored to file')
        fobj = open(filename, 'w')
        write_str = "[" + str(self.sld1.value()) + ","\
                    + str(self.sld1.value())\
                    + "] \n"
        fobj.write(write_str)
        fobj.close()
 
    def SlotReadPosition(self):
        self.lblStatus.setText(' Pose read from file ')
        # Den vorgegebenen Pfad einlesen, jede Zeile ein Goal
        with open(filename, 'r') as fin:
            for line in fin:
                self.path.append(eval(line))  # Goal anhaengen
        del self.path[0]  # [0, 0] entfernen (ertes Element )
        rospy.loginfo(str(self.path))
        # setze Slider mit den Werten aus der Datei
        self.sld1.setValue(self.path[0][0])
        self.sld1.setValue(self.path[0][1])


if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        ui = UIClass()
        sys.exit(app.exec_())

    except rospy.ROSInterruptException:
        pass

