#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.core import planning_scene
from moveit.task_constructor import core, stages
from moveit.python_tools import roscpp_init
from moveit_commander import PlanningScene
import time
from geometry_msgs.msg import PoseStamped
import time

task = core.Task()

# start from current robot state
task.add(stages.CurrentState("current state"))

# Cartesian motion along x
move = stages.MoveRelative("x +0.2", core.CartesianPath())
move.group = group
dir = Vector3Stamped(header=Header(frame_id = "world"),
vector=Vector3(0.2,0,0))
move.setDirection(dir)
task.add(move)