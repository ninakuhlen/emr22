#!/usr/bin/env python
# pick_and_place_collision_v2.py
# V2 OHNE  TCP-Goal und IK - veraltet
# ------------------------------------
# edited WHS, OJ , 20.2.2022 #
# -------------------------------------
# Pick and Place
# in Python mit der move_group_api
# und Kollsionsverhütung
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


def wait_for_state_update(box_name, scene, box_is_known=False,
                          box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0
        is_known = box_name in scene.get_known_object_names()
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
        rospy.sleep(0.1)
        seconds = rospy.get_time()
    return False


# First initialize moveit_ Command and rospy nodes:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
# Instantiate the robot commander object,
# which is used to control the whole robot
robot = moveit_commander.RobotCommander()

# Instantiate the MoveGroupCommander object.
group_name = "ur5_arm"
group = moveit_commander.MoveGroupCommander(group_name)
group_name_gripper = "gripper"
group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)

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
# print(robot.get_current_state())
print("")

# Create an object for PlanningSceneInterface.
print("=== Adding Desktop-Plate to Planning Scene  ===")
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2.0)
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame() 
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.z = -0.2
box_name = "desktop"
scene.add_box(box_name, box_pose, size=(2.0, 2.0, 0.05))
rospy.loginfo(wait_for_state_update(box_name, scene, box_is_known=True))

# ---- 1. Move to home position ----
# input(" Go to Home Position => Enter")
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
# input("\n Open Gripper => Enter")
joint_gripper = group_gripper.get_current_joint_values()
print("Gripper Angle is", joint_gripper)
joint_gripper[0] = 0.001  # complete open is 0.0007  closed is pi/4
group_gripper.go(joint_gripper, wait=True)
group_gripper.stop()

# --- 3. Place the TCP above the blue box
# input(" Go close to blue box => Enter")
# ---> Goal with Jointangles
# abgelesen nach Handfahrt
joint_goal = group.get_current_joint_values()
joint_goal[0] = np.deg2rad(47)       # shoulder pan
joint_goal[1] = np.deg2rad(-55)      # shoulder lift
joint_goal[2] = np.deg2rad(111)      # elbow
joint_goal[3] = np.deg2rad(-151)     # wrist1
joint_goal[4] = np.deg2rad(-87)      # wrist2
joint_goal[5] = np.deg2rad(50)       # wrist3
group.go(joint_goal, wait=True)
print("Reached TCP Goal above blue box", joint_goal)


# --- 4. Move the TCP close to the object 0.2m down
# ============== Cartesian Paths ==============
print("plan a cartesion path")
# The Cartesian path is directly planned by specifying
# the waypoints list through which the end effector passes
waypoints = []

wpose = group.get_current_pose().pose
wpose.position.z = -0.13  # First move down (z)
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

# Publish
display_trajectory_publisher.publish(display_trajectory)

# ==== Execute the calculated path:
# print("the plan is", plan)
print("execute plan ")
group.execute(plan, wait=True)


# --- 5. Close the gripper
input("\n Close Gripper => Enter")
joint_gripper = group_gripper.get_current_joint_values()
print("Gripper Angle is", joint_gripper)
joint_gripper[0] = pi/11  # complete open is 0.0007  closed is pi/4
group_gripper.go(joint_gripper, wait=True)
group_gripper.stop()

# --- 6. Move the TCP close to the object 0.2m up
input(" Lift blue box => Enter")
# ============== Cartesian Paths ==============
print("plan a cartesion path")
# The Cartesian path is directly planned by specifying
# the waypoints list through which the end effector passes
waypoints = []

wpose = group.get_current_pose().pose
wpose.position.z = 0.2  # First move down (z)
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

# Publish
display_trajectory_publisher.publish(display_trajectory)

# ==== Execute the calculated path:
# print("the plan is", plan)
print("execute plan ")
group.execute(plan, wait=True)

# --- 7. Move the TCP above the plate
# input(" Go to Home Position => Enter")
joint_goal = group.get_current_joint_values()
joint_goal[0] = np.deg2rad(111)       # shoulder pan
joint_goal[1] = np.deg2rad(-52)      # shoulder lift
joint_goal[2] = np.deg2rad(101)      # elbow
joint_goal[3] = np.deg2rad(-121)     # wrist1
joint_goal[4] = np.deg2rad(-84)     # wrist2
joint_goal[5] = np.deg2rad(50)      # wrist3
group.go(joint_goal, wait=True)
print("Reached Goal above plate", joint_goal)

# --- 8. Open the gripper
# input("\n Open Gripper => Enter")
joint_gripper = group_gripper.get_current_joint_values()
print("Gripper Angle is", joint_gripper)
joint_gripper[0] = 0.001  # complete open is 0.0007  closed is pi/4
group_gripper.go(joint_gripper, wait=True)
group_gripper.stop()

# --- at the end -----
#input("Remove Box")  # Otherwise it will stay
scene.remove_world_object(box_name)
