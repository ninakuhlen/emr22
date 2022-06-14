#!/usr/bin/env python3
# grasp_detected_object_tf.py
# 
# ------------------------------------
# edited WHS, OJ , 14.6.2022 #
# -------------------------------------
# Pick and Place
# in Python mit der move_group_api
# und Kollsionsverhütung
# -----------------------------------------
#
# http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
# -----------------------------------------
# usage
#   ...
# ----------------------------------------------------------------
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import numpy as np  # für deg2rad
# from tf import TransformListener
import tf


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
group_name = "ur3_arm"
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
# print("= Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
# print("= End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
# print("= Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
# print("= Printing robot state")
# print(robot.get_current_state())
# print("")

# create tf-listener
#t = TransformListener()
listener = tf.TransformListener()


# ---- 1. Move to home position ----
input("confirm moving ur3_arm to home position")
joint_goal = group.get_named_target_values("home")
group.go(joint_goal, wait=True)
# print("Reached Joint Goal Home", joint_goal)

# --- 3. get the pose of object_X  from Camera
print("looking for object_9 ==>")

 # if listener.frameExists("world") and listener.frameExists("object_8"):
(trans,rot) = listener.lookupTransform('/world', '/object_9', rospy.Time())
print("detected object position")
# print(rot)
print(trans)
# else:
#    print("  no object_9 found => exit() ")
#    exit()

# --- 4. go to object position
# rosrun tf tf_echo world robotiq_85_left_finger_link
# trans = [0.36, 0.08, 0.3]  # Test Position 
print("translation is ",trans)
pose_goal = group.get_current_pose()
pose_goal.pose.position.x  = trans[0] -0.04 # from tf-Tree
pose_goal.pose.position.y  = trans[1] -0.02 # from tf-Tree
pose_goal.pose.position.z  = trans[2]  # from tf-Tree
pose_goal.pose.position.z = 0.35  # 35cm higher
print(" going to ", pose_goal.pose.position)
input("confirm moving ur3_arm to this position")
group.set_pose_target(pose_goal)
plan = group.plan()
sucess = group.go(wait=True)
print("suc?", sucess)
group.stop()
group.clear_pose_targets()

input("confirm turning wrist3")
joint_goal = group.get_current_joint_values()
joint_goal[5] = np.deg2rad(45)       # wrist3
group.go(joint_goal, wait=True)


print("plan a cartesion path")
waypoints = []
wpose = group.get_current_pose().pose
wpose.position.z = -0.3  # First move down (z)
waypoints.append(copy.deepcopy(wpose))
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
input("confirm moving ur3_arm 10cm deeper")
group.execute(plan, wait=True)

input("confirm moving ur3_arm to home position")
joint_goal = group.get_named_target_values("home")
group.go(joint_goal, wait=True)

# --- at the end -----

