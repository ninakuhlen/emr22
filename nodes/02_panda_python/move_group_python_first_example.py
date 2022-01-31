#!/usr/bin/env python
# move_group_python_first_example.py
# ------------------------------------
# edited WHS, OJ , 31.1.2022 #
# -------------------------------------
# Erste einfache Bewegungen des Panda-Arms
# unter Nutzung des
# move_group_python_interface
# -----------------------------------------
# basiert auf dem Code 
# https://www.fatalerrors.org/a/moveit-learning-notes-move-group-python-interface.html
# -----------------------------------------
# usage
#   $1 roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
#   $2 rosrun emr22 move_group_python_first_example.py
# ------------------------------------------------------------------
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
group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

# Create a Publisher.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

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
print(robot.get_current_state())
print("")

# ============== Plan to a joint goal  =============
# Gelenkwinkel in rad
joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = pi/3
joint_goal[6] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
print("Going to Joint Goal Angles", joint_goal)
group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group.stop()
input(" Joint Goal reached ")

# ============== Plan to a joint goal  =============
# Gelenkwinkel in DEG (Joints 1..7)
joint_goal = group.get_current_joint_values()
joint_goal[0] = np.deg2rad(42)
joint_goal[1] = np.deg2rad(34)
joint_goal[2] = np.deg2rad(29)
joint_goal[3] = np.deg2rad(-87)
joint_goal[4] = np.deg2rad(-12)
joint_goal[5] = np.deg2rad(134)
joint_goal[6] = np.deg2rad(148)

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
print("Going to Joint Goal Angles 2", joint_goal)
group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group.stop()
input(" Joint Goal reached ")


# ============== Planning to a Pose Goal =============
# Define an attitude target in m vom panda_link0
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = 0.77
pose_goal.orientation.y = 00.61
pose_goal.orientation.z = -0.09
pose_goal.orientation.w = -0.12
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4
group.set_pose_target(pose_goal)

# Call planner to calculate the plan and execute it
print("Going to Pose Goal \n", pose_goal)
plan = group.go(wait=True)

# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
input(" Pose Goal reached ")

# ============== Cartesian Paths ==============
print("plan a cartesion path")
# The Cartesian path is directly planned by specifying
# the waypoints list through which the end effector passes
waypoints = []

wpose = group.get_current_pose().pose
wpose.position.z = 0.2  # First move up (z)
wpose.position.y += 0.1  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += 0.2  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= 0.2  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.orientation.x += pi  # Turn
waypoints.append(copy.deepcopy(wpose))

wpose.orientation.y += pi  # Turn
waypoints.append(copy.deepcopy(wpose))

wpose.orientation.z += pi  # Turn
waypoints.append(copy.deepcopy(wpose))

wpose.orientation.w += pi  # Turn
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold
# by setting it to 0.0 disabling:
(plan, fraction) = group.compute_cartesian_path(
                                                waypoints,
                                                0.01,        # eef_step
                                                0.0)         # jump_threshold
# Displaying a Trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
input(" Display Trajectory")

# Publish
display_trajectory_publisher.publish(display_trajectory)

# ==== Execute the calculated path:
# print("the plan is", plan)
print("execute plan ")
group.execute(plan, wait=True)
input(" Plan executed ")
