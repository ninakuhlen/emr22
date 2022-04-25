#!/usr/bin/env python3
# -- starthilfe_ur5.py --
# GUI to control all the Launch Files etc. in the emr22 course
# edited WHS, OJ , 21.2.2022 #

from PyQt5.QtWidgets import (QWidget,QApplication,  QPushButton)
from PyQt5.QtCore import Qt
import sys
import os


class MainWindow(QWidget):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        # --- roscore ---
        self.myPb_roscore = QPushButton(self)
        self.myPb_roscore.setText('Starte ROS-Master')
        self.myPb_roscore.setGeometry(10, 10, 300, 40)  # x,y,w,h
        self.myPb_roscore.clicked.connect(self.slot_roscore)

        # --- roslaunch ---
        self.myPb_gazebo_ur5 = QPushButton(self)
        self.myPb_gazebo_ur5.setText('Starte UR5 in Gazebo und MoveIt!')
        self.myPb_gazebo_ur5.setGeometry(10, 50, 300, 40)  # x,y,w,h
        self.myPb_gazebo_ur5.clicked.connect(self.slot_ur5)

        self.myPb_depth = QPushButton(self)
        self.myPb_depth.setText('Starte UR5, Gazebo, Moveit mit Depth-Cam')
        self.myPb_depth.setGeometry(10, 90, 300, 40)  # x,y,w,h
        self.myPb_depth.clicked.connect(self.slot_ur5_depth)

        # --- Pick and Place Script ---
        self.myPb_pick_place = QPushButton(self)
        self.myPb_pick_place.setText('PyScript: MoveItAPI - Pick + Place')
        self.myPb_pick_place.setGeometry(10, 130, 300, 40)  # x,y,w,h
        self.myPb_pick_place.clicked.connect(self.slot_pick_place)

        # --- Pick and Place Script ---
        self.myPb_pick_place_dc = QPushButton(self)
        self.myPb_pick_place_dc.setText('PyScript: PP with Depth Cam')
        self.myPb_pick_place_dc.setGeometry(10, 170, 300, 40)  # x,y,w,h
        self.myPb_pick_place_dc.clicked.connect(self.slot_pick_place_depth_cam)

        # --- find_object_2D ---
        self.myPb_find_object_2D = QPushButton(self)
        self.myPb_find_object_2D.setText('find_object_2D')
        self.myPb_find_object_2D.setGeometry(10, 210, 300, 40)  # x,y,w,h
        self.myPb_find_object_2D.clicked.connect(self.slot_find_object_2D)

        # --- Window konfigurieren und starten
        self.setGeometry(300, 300, 400, 300)
        self.setWindowTitle('EMR22 - Starthilfe UR5e ')
        self.show()

    # --- Die  Slot-Methoden ---
    def slot_roscore(self):
        os.system('gnome-terminal -- bash -c "roscore; exec bash"')

    def slot_ur5(self):
        os.system('gnome-terminal -- bash -c "roslaunch ur5_gripper_moveit_config demo_gazebo_pick_and_place.launch; exec bash"')

    def slot_ur5_depth(self):
        os.system('gnome-terminal -- bash -c "roslaunch ur5_gripper_moveit_config demo_gazebo_pick_and_place_depth.launch; exec bash"')

    def slot_pick_place(self):
        os.system('gnome-terminal -- bash -c "rosrun emr22 pick_and_place_collision.py; exec bash"')

    def slot_pick_place_depth_cam(self):
        os.system('gnome-terminal -- bash -c "rosrun emr22 pick_and_place_collision_depth_cam.py; exec bash"')
    
    def slot_find_object_2D(self):
        os.system('gnome-terminal -- bash -c "roslaunch emr22 find_object_2d.launch"')


if __name__ == '__main__':

    app = QApplication(sys.argv)
    mw = MainWindow()
    sys.exit(app.exec_())
