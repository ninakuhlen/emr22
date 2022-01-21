# movegroup_API.py
# Python Programm um den UR3-Arm per MoveIt zu bewegen
# https://python.hotexamples.com/de/examples/moveit_commander/MoveGroupCommander/get_end_effector_link/python-movegroupcommander-get_end_effector_link-method-examples.html

def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("moveit_demo")

        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander("right_arm")

        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)

        # Allow some leeway in position(meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.02)
        right_arm.set_goal_orientation_tolerance(0.1)

        # Start the arm in the "resting" pose stored in the SRDF file
        right_arm.set_named_target("resting")
        right_arm.go()

        # Move to the named "straight_forward" pose stored in the SRDF file
        right_arm.set_named_target("straight_forward")
        right_arm.go()
        rospy.sleep(2)

        # Move back to the resting position at roughly 1/4 speed
        right_arm.set_named_target("resting")

        # Get back the planned trajectory
        traj = right_arm.plan()

        # Scale the trajectory speed by a factor of 0.25
        new_traj = scale_trajectory_speed(traj, 0.25)

        # Execute the new trajectory
        right_arm.execute(new_traj)
        rospy.sleep(1)

        # Exit MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)
