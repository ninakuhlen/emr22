#!/usr/bin/env python3
# EMR22 - ProfJust
# based on ROS Developers LIVE-Class #59: MoveIt! Robot Perception
# https://www.youtube.com/watch?v=E71pb2H4VDQ
#----------------------------------------------------------------------
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: 

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf import TransformListener

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_interface_emr22', anonymous=True)
robot = moveit_commander.RobotCommander()


input("confirm moving ur3_arm to home position")
# Put the arm in the home position
arm_group = moveit_commander.MoveGroupCommander("ur3_arm")
arm_group.set_named_target("home")
plan1 = arm_group.go()

#  1)  put the arm to first grasping position
input("confirm moving ur3_arm to first grasping position")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 1.0
pose_target.position.y = 1.0
pose_target.position.z = 1.0
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

#  2)  put the arm 15cm higher
input("confirm moving ur3_arm to first grasping position +15cm")
pose_target.position.z += 0.15
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()

# get the pose of object_X  from Camera
t = TransformListener()
if t.frameExists("/world") and t.frameExists("/object_8"):
    (translation, rotation) = t.lookupTransform("world", "object_8", rospy.Time())
    print("detected object position")
    print(t)
    print(translation)
else:
    exit()


#  3)  put the arm to object
input("confirm moving ur3_arm to detected object position")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position = translation  # from tf-Tree
pose_target.position.z -= 0.15  # 15cm higher than object
arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()


