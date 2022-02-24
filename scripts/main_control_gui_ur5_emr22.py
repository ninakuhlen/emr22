#!/usr/bin/env python3
# -- main_control_gui_ur5_emr22.p --
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

        # --- Pick and Places Sript ---
        self.myPb_pick_place = QPushButton(self)
        self.myPb_pick_place.setText('PyScript: MoveItAPI - Pick + Place')
        self.myPb_pick_place.setGeometry(10, 90, 300, 40)  # x,y,w,h
        self.myPb_pick_place.clicked.connect(self.slot_pick_place)

        # --- Pick and Place Script ---
        self.myPb_pick_place_dc = QPushButton(self)
        self.myPb_pick_place_dc.setText('PyScript: PP with Depth Cam')
        self.myPb_pick_place_dc.setGeometry(10, 130, 300, 40)  # x,y,w,h
        self.myPb_pick_place_dc.clicked.connect(self.slot_pick_place_depth_cam)

        # --- Window konfigurieren und starten
        self.setGeometry(300, 300, 600, 400)
        self.setWindowTitle('EMR22 - main-control-gui')
        self.show()

    # --- Die  Slot-Methoden ---
    def slot_roscore(self):
        os.system('gnome-terminal -- bash -c "roscore; exec bash"')

    def slot_ur5(self):
        os.system('gnome-terminal -- bash -c "roslaunch ur5_gripper_moveit_config demo_gazebo_pick_and_place.launch; exec bash"')

    def slot_pick_place(self):
        os.system('gnome-terminal -- bash -c "rosrun emr22 pick_and_place_collision.py; exec bash"')

    def slot_pick_place_depth_cam(self):
        os.system('gnome-terminal -- bash -c "rosrun emr22 pick_and_place_collision_depth_cam.py; exec bash"')

if __name__ == '__main__':

    app = QApplication(sys.argv)
    mw = MainWindow()
    sys.exit(app.exec_())


"""# --- Slider erstellen -----
        self.mySlider = QSlider(Qt.Horizontal, self)

        #mySlider = mySliderClass()
        self.mySlider.setFocusPolicy(Qt.NoFocus)
        self.mySlider.setGeometry(30, 40, 180, 30) #x,y,w,h
        self.mySlider.setValue(20)

        # --- LCD konstruieren -----
        self.myLcd=QLCDNumber(2, self)
        self.myLcd.setGeometry(60, 100, 80, 50) #x,y,w,h
        self.myLcd.display(20)

        # Verbinden des Signals valueChanged
        # mit der Slot-Funktion myLcd.display
        self.mySlider.valueChanged[int].connect(self.myLcd.display)

        #--- zwei PushButtons
        myPBmore = QPushButton(self)
        myPBmore.setText('>')
        myPBmore.setGeometry(0, 0, 40, 40) #x,y,w,h
        myPBmore.clicked.connect(self.plus)

        self.myPBless = QPushButton(self)
        self.myPBless.setText('<')
        self.myPBless.setGeometry(180, 0, 40, 40) #x,y,w,h
        self.myPBless.clicked.connect(self.minus)


        #--- Window konfigurieren
        self.setGeometry(300, 300, 280, 170)
        self.setWindowTitle('Robotik Bocholt - Slider LCD')
        self.show()

    #--- Die beiden Slot-Methoden
    def plus(self):
        wert =self.mySlider.value()  #Slider Wert holen
        wert =self.wert+1
        mySlider.setValue(wert) #Slider Wert setzen
    def minus(self):
        wert = self.mySlider.value()
        wert =wert-1
        self.mySlider.setValue(wert)"""