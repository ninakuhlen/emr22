#!/usr/bin/env python
# pick_and_place.py
# ------------------------------------
# edited WHS, OJ , 16.2.2022 #
# -------------------------------------
# Pick and Place
# in Python mit der move_group_api
# -----------------------------------------
# basiert auf dem Code
# https://roboticscasual.com/ros-tutorial-pick-and-place-task-with-the-moveit-c-interface/
# -----------------------------------------
# usage
#   $1 roslaunch ur5_gripper_moveit_config demo_gazebo_pick_and_place.launch
# 
#   $2 rosrun emr22 pick_and_place.py
# ----------------------------------------------------------------
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import numpy as np  # für deg2rad

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

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("= Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("= Printing robot state")
# print(robot.get_current_state())
print("")

# ---- 1. Move to home position ----
### Error: Controller 'ur5_arm_controller' failed with error GOAL_TOLERANCE_VIOLATED: 
### ABORTED: Solution found but controller failed during execution

input(" Go to Home Position => Enter")
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0.001           # shoulder pan
joint_goal[1] = -pi/2       # shoulder lift
joint_goal[2] = pi/2        # elbow
joint_goal[3] = -pi/2       # wrist1
joint_goal[4] = -pi/2       # wrist2
joint_goal[5] = 0.001          # wrist3
group.go(joint_goal, wait=True)
print("Reached Joint Goal Home", joint_goal)

# --- 2. Open the gripper
input("\n Open Gripper => Enter")
joint_gripper = group_gripper.get_current_joint_values()
print("Gripper Angle is", joint_gripper)
joint_gripper[0] = 0.001  # complete open is 0.0007  closed is pi/4
group_gripper.go(joint_gripper, wait=True)
group_gripper.stop()

# --- 3. Place the TCP above the blue box
# ---> Goal with Orientation
input(" Go close to blue box => Enter")
pose_goal = geometry_msgs.msg.Pose()
"""#  Orientierung waagerecht nach links
pose_goal.orientation.x = 0.77
pose_goal.orientation.y = 00.61
pose_goal.orientation.z = 0.09
pose_goal.orientation.w = -0.12"""
#  Orientierung ?? Fahrzeug
pose_goal.orientation.x = 0
pose_goal.orientation.y = 0
pose_goal.orientation.z = 0
pose_goal.orientation.w = -1
pose_goal.position.x = 0.3
pose_goal.position.y = 0.5
pose_goal.position.z = 0.2
group.set_pose_target(pose_goal)
# Call planner to calculate the plan and execute it
print("Going to Pose Goal \n", pose_goal)
plan = group.go(wait=True)
group.stop()

# --- 4. Move the TCP close to the object 0.2m down
input(" Go to grip-position => Enter")
pose_goal.position.z = pose_goal.position.z - 0.2
group.set_pose_target(pose_goal)
plan = group.go(wait=True)
group.stop()






# target_pose1.position.z = target_pose1.position.z - 0.2;

# --- 5. Open the gripper

# --- 6. Move the TCP above the plate

# target_pose1.position.z = target_pose1.position.z + 0.2;
#    target_pose1.position.x = target_pose1.position.x - 0.6;


# --- 7. Lower the TCP above the plate
# target_pose1.position.z = target_pose1.position.z - 0.14;

# --- 8. Open the gripper

# ---- 9. Move to home position ----
input(" Go to Home Position => Enter")
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0           # shoulder pan
joint_goal[1] = -pi/2       # shoulder lift
joint_goal[2] = pi/2        # elbow
joint_goal[3] = -pi/2       # wrist1
joint_goal[4] = -pi/2       # wrist2
joint_goal[5] = 0           # wrist3
group.go(joint_goal, wait=True)
print("Reached Joint Goal Home", joint_goal)



