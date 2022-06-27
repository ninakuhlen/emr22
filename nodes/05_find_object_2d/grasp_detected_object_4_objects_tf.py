#!/usr/bin/env python3
# grasp_detected_object_tf.py
# 
# ------------------------------------
# edited WHS, OJ , 21.6.2022 #
# -------------------------------------
# Pick and Place für den realen UR3
# in Python mit der move_group_api
# und Kollsionsverhütung mit der Realsense 3D-Kamera
# -----------------------------------------
#
# http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
# -----------------------------------------
# usage
#   ...
# ----------------------------------------------------------------
from inspect import Traceback
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np  # für deg2rad
from math import pi
import tf
trans2 = [0,0,0]

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

# Create a Publisher
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20)

# create tf-listener
listener = tf.TransformListener()

# ##### Getting Basic Information ###############
# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
# print("= Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("= End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()

# ---- 1. Move to home position ----
input("confirm moving ur3_arm to home position")
joint_goal = group.get_named_target_values("home")
group.go(joint_goal, wait=True)

#-------------------------------------------------------------------------------------------
def object_8():
    input("object_8")
    try:
        (trans, rot) = listener.lookupTransform('/world', '/object_8',rospy.Time())
        print("object 8 found")
        print(trans)
        trans2[0]=trans[0]
        trans2[1]=trans[1]
        trans2[2]=trans[2]
        print(trans2)
    except:
        pass
    else: return trans,rot

def object_9():
    input("object_9")
    try:
        (trans, rot) = listener.lookupTransform('/world', '/object_9',rospy.Time())
        print("object 9 found")
        print(trans)
        trans2[0]=trans[0]
        trans2[1]=trans[1]
        trans2[2]=trans[2]
        print(trans2)
    except:
        pass
    else: return trans,rot

def object_10():
    input("object_10")
    try:
        (trans, rot) = listener.lookupTransform('/world', '/object_10',rospy.Time())
        print("object 10 found")
        print(trans)
        trans2[0]=trans[0]
        trans2[1]=trans[1]
        trans2[2]=trans[2]
        print(trans2)
    except:
        pass
    else: return trans,rot
def object_11():
    input("object_11")
    try:
        (trans, rot) = listener.lookupTransform('/world', '/object_11',rospy.Time())
        print("object 11 found")
        print(trans)
        trans2[0]=trans[0]
        trans2[1]=trans[1]
        trans2[2]=trans[2]
        print(trans2)
    except:
        pass
    else: return trans,rot
#--------------------------------------------------------------------------------------------


# --- 2. get the pose of object_X  from Camera
print("looking for objects ==>")
input("confirm looking for objects")
try:
    object_8() or object_9() or object_10() or object_11()

except:
    pass

print("trans2")


# --- 3. Object in Scene eintragen
# Desktop Plate is not needed because of Depth Cam, Yes!
# But Gripper does not go to Desktop then =>
# Add the object to the planning scene and deactivate collision checking
print("=== Adding object to Planning Scene  ===")
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(1.0)  # Wichtig! ohne Pause funkts nicht
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = trans2[0]
box_pose.pose.position.y = trans2[1]
box_pose.pose.position.z = trans2[2]
box_name = "object"
scene.add_box(box_name, box_pose, size=(0.08, 0.08, 0.08))
rospy.loginfo(wait_for_state_update(box_name, scene, box_is_known=True))


print("=== Adding Wall object to Planning Scene  ===")
wall_pose = geometry_msgs.msg.PoseStamped()
wall_pose.header.frame_id = robot.get_planning_frame()

# RViZ export 0 0 -0.342898 0.939373
wall_pose.pose.orientation.x = 0.0
wall_pose.pose.orientation.y = 0.0
wall_pose.pose.orientation.z = -0.342898
wall_pose.pose.orientation.w = 0.939373

wall_pose.pose.position.x = 0.1 
wall_pose.pose.position.y = -0.5
wall_pose.pose.position.z = 0.3
wall_name = "wall"
scene.add_box(wall_name, wall_pose, size=(1.0, 0.05, 1.0))
rospy.loginfo(wait_for_state_update(wall_name, scene, box_is_known=True))

# --- 4. go to object position and turn gripper
# rosrun tf tf_echo world robotiq_85_left_finger_link
# trans = [0.36, 0.08, 0.3]  # Test Position
print("translation is ", trans2)
pose_goal = group.get_current_pose()
pose_goal.pose.position.x = trans2[0] - 0.02  # from tf-Tree
pose_goal.pose.position.y = trans2[1] - 0.02  # from tf-Tree
pose_goal.pose.position.z = trans2[2]  # from tf-Tree
pose_goal.pose.position.z = 0.35  # 35cm high

# Gripper soll um 45° Drehen um Object zu Greifen
# da uns die Angabe in Quaternionen schwer fällt, hier ein 
# Umweg über Euler
# Drehwinkel um den Vektor ist pi/4 = 0.7853..
print(" going to pose", pose_goal.pose.position)
print(" going to orientation quaternion ", pose_goal.pose.orientation)
# pose_goal.pose.orientation ist ein Tupel => nicht veränderbar
# => Variable quaternion instanzieren
quaternion = (
    pose_goal.pose.orientation.x,
    pose_goal.pose.orientation.y,
    pose_goal.pose.orientation.z,
    pose_goal.pose.orientation.w)
# quat 2 eul für bessere Lesbarkeit
euler_goal_pose_orientation = tf.transformations.euler_from_quaternion(quaternion) 
print(" going to orientation euler", euler_goal_pose_orientation)

# pi/4 addieren und in Quaternion zurück
quaternion = tf.transformations.quaternion_from_euler(
    euler_goal_pose_orientation[0],
    euler_goal_pose_orientation[1],
    euler_goal_pose_orientation[2] + pi/4,
    axes='sxyz'  )

# WICHTIG! Ergebnisse => Goal Pose
pose_goal.pose.orientation.x = quaternion[0]
pose_goal.pose.orientation.y = quaternion[1]
pose_goal.pose.orientation.z = quaternion[2]
pose_goal.pose.orientation.w = quaternion[3]    

# Debug Ausgabe
euler_goal_pose_orientation = tf.transformations.euler_from_quaternion(quaternion) 
print(" going to orientation euler neu", euler_goal_pose_orientation)

input("confirm moving ur3_arm to this position")
group.set_pose_target(pose_goal)
plan = group.plan()
sucess = group.go(wait=True)
print("suc?", sucess)
# joint_goal = group.get_current_joint_values()
# joint_goal[5] = np.deg2rad(0)  # turn wrist3
# group.go(joint_goal, wait=True)

# --- 6. go to grip position 
print("plan a cartesion path")
waypoints = []
wpose = group.get_current_pose().pose
wpose.position.z -= 0.11  # move down (z)
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = group.compute_cartesian_path(
                                                waypoints,
                                                0.001,        # eef_step
                                                0.0)         # jump_threshold
# Displaying a Trajectory in RViZ
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

# Execute the calculated path:
input("confirm moving ur3_arm 9cm deeper")
group.execute(plan, wait=True)
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

# --- 7. Gripping

# für die  Trajektorienplanung das Object an den Gripper attachen
grasping_group = "gripper"
touch_links = ['robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link'] 
# robot.get_link_names(group=grasping_group) => ['robotiq_85_left_knuckle_link']
print(touch_links)
# touch links (collision allowed) should be 
# robotiq_85_left_finger_tip_link  and
# robotiq_85_right_finger_tip_link

#  no worxxxxxxxxxxx  
scene.attach_box('robotiq_85_left_finger_tip_link', box_name, touch_links=touch_links)
rospy.loginfo(wait_for_state_update(box_name, scene, box_is_known=True))
input("Confirm: you have to close the gripper with the UR3-Teach-Pad EA Werkzeugausgang 0 auf OFF")

# --- 8. go to home position
input("confirm moving ur3_arm to home position")
joint_goal = group.get_named_target_values("home")
group.go(joint_goal, wait=True)
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

# --- at the end -----

scene.remove_attached_object('robotiq_85_left_finger_tip_link', name=box_name)
scene.remove_world_object(box_name)
scene.remove_world_object(wall_name)
input("Remove Box and Wall")  # Otherwise it will stay
