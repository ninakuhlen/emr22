#testMoveIt

from moveit_commander import MoveGroupCommander
cmd = MoveGroupCommander(ur3)
cmd.set_pose_target(pose)  # to move the endeffector into a specific pose, OR
cmd.set_joint_value_target(joint_values) # if you know the positions the joints should be in
cmd.go()
