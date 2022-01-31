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
#       use moveit_panda_demo.rviz
#
#   $2 rosrun emr22 move_group_python_first_example.py
# ----------------------------------------------------------------
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

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
display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path',
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


# Add a cube to the scene and set the location in panda_leftfinger
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = 0.5
box_pose.pose.position.y = 0.1
box_pose.pose.position.z = 0.05  # half of size
box_name = "box1"
scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

input(" box added")

# EEF go to Box
# ============== Planning to a Pose Goal =============
# Define an attitude target in m vom panda_link0
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = 1.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = 0.0
pose_goal.orientation.w = 0.0

pose_goal.position.x = 0.5
pose_goal.position.y = 0.1
pose_goal.position.z = 0.3

group.set_pose_target(pose_goal)

# Call planner to calculate the plan and execute it
print("Going to Pose Goal \n", pose_goal)
plan = group.go(wait=True)

# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
input(" Pose Goal at Box reached?")

# ---- Attach Box to Hand ----
grasping_group = 'hand'
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(eef_link, box_name, touch_links=touch_links)


# EEF go to Goal
# ============== Planning to a Pose Goal =============
# Define an attitude target in m vom panda_link0
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = 1.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = 0.0
pose_goal.orientation.w = 0.0

pose_goal.position.x = -0.5
pose_goal.position.y = -0.1
pose_goal.position.z = 0.25

group.set_pose_target(pose_goal)

# Call planner to calculate the plan and execute it
print("Going to Pose Goal \n", pose_goal)
plan = group.go(wait=True)

# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
input(" Pose Goal reached?")


scene.remove_attached_object(eef_link, name=box_name)


# EEF go to Goal
# ============== Planning to a Pose Goal =============
# Define an attitude target in m vom panda_link0
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = 1.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = 0.0
pose_goal.orientation.w = 0.0

pose_goal.position.x = -0.3
pose_goal.position.y = 0.3
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

input("Remove Box? \n", pose_goal)
scene.remove_world_object(box_name)
