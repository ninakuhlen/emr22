# ur5e_gripper_per_socket.py
# ----------------------------------------------
# Python Programm zum Öffnen/Schließen des Gripper
# Author: Marius Andrae
# siehe auch    aumax Posts: 5 Apprentice 
# https://dof.robotiq.com/discussion/2420/control-robotiq-gripper-mounted-on-ur-robot-via-socket-communication-python
# date: 21.7.2022 @ w-hs 
# ---------------------- what's it for? ------------------
# Ur5e goes to 2 Positions and opens/closes the gripper
# ---------- Configuration -------------------------------
# real UR5e with robotiq 2f-Gripper, directly cabled to the tool connector
# uses no Modbus or socat
# UR5e runs external_control - programm
# in automatic-mode and wiggeshoff config
# ---------------------------------------------
# remote PC 
# $1 roscore
# $2 roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100 kinematics_config:=$(rospack find ur_calibration)/etc/ur5e_robot_calibration.yaml
# $3 roslaunch ur5_gripper_moveit_config demo_real_ur5e.launch sim:=false limited:=true
# $4  
# -------- gripper per socket ------------------
import socket  # Verbindung vom PC zur UR5e-Control Box
import time    # Pausen sind nötig
# -----------------------------------------------
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import numpy as np

HOST = "192.168.0.100" # IP-Adresse des UR5
PORT1 = 30002 # ???
PORT2 = 29999 # ???
PORT3 = 63352 # Port zum Gripper

#----- Moveit API  - Python MoveGroupApi ----
Variable = 0
# First initialize moveit_ Command and rospy nodes:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
# Instantiate the robot commander object,
# which is used to control the whole robot
robot = moveit_commander.RobotCommander()

# Create an object for PlanningSceneInterface.
scene = moveit_commander.PlanningSceneInterface()

# Instantiate the MoveGroupCommander object.
group_name = "ur5_arm"
group = moveit_commander.MoveGroupCommander(group_name)
group_name_gripper = "gripper"
group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)

# Create a Publisher.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
print("============ Waiting for RVIZ...")
rospy.sleep(2)

# ##### Getting Basic Information ###############
# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("= Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("= End effector: %s" % eef_link)

# robot:
print("= Printing robot state")
# print(robot.get_current_state())
print("")

# ---- 1. Move to home position ----
### Error: Controller 'ur5_arm_controller' failed with error GOAL_TOLERANCE_VIOLATED: 
### ABORTED: Solution found but controller failed during execution

while Variable == 0:

# ========= Socket zum Gripper ============================
    # Hole die Positionsdaten vom Gripper,
    # sollte noch an ROS - Robot State gesendet werden
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
        s.connect((HOST,PORT3))
        s.sendall(b'GET POS\n')
        data=s.recv(2**10)

    print('Gripper finger position is: ',data)
    time.sleep(1) # wichtig 

    # sendet Befehl

# ==========================================================
# weiter wie bisher    
    #input(" Go to Home Position => Enter")
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = (0/180)*pi          # shoulder pan
    joint_goal[1] = (-88/180)*pi       # shoulder lift
    joint_goal[2] = (89/180)*pi        # elbow
    joint_goal[3] = (-90/180)*pi       # wrist1
    joint_goal[4] = (-90/180)*pi       # wrist2
    joint_goal[5] = (-0/180)*pi           # wrist3
    group.go(joint_goal, wait=True)
    print("Reached Joint Goal Home", joint_goal)

# ========= Socket zum Gripper ============================
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
        s.connect((HOST,PORT3))
        s.sendall(b'SET POS 50\n')      # Übertragung der Socket-Message
        data=s.recv(2**10)
    time.sleep(1)
# ==========================================================
    #input(" Go to Home Position => Enter")
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = (10/180)*pi          # shoulder pan
    joint_goal[1] = (-88/180)*pi       # shoulder lift
    joint_goal[2] = (89/180)*pi        # elbow
    joint_goal[3] = (-90/180)*pi       # wrist1
    joint_goal[4] = (-90/180)*pi       # wrist2
    joint_goal[5] = (-0/180)*pi           # wrist3
    group.go(joint_goal, wait=True)
    print("Reached Joint Goal Home", joint_goal)
    time.sleep(1)
