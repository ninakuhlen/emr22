MoveIt Tutorial mit Panda Arm, Python API

$1 roslaunch panda_moveit_config demo.launch rviz_tutorial:=true

$2 run emr22 move_group_python_first_example.py 

    Fährt im RVIZ mehrere Positionen an


UR5 ohne Gripper für PID

$ roslaunch ur5-joint-position-control ur5_gazebo_joint_position_control.launch


UR5 und MoveIt ohne Gripper

    roslaunch ur_gazebo ur5.launch 
    roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
    roslaunch ur5_moveit_config demo.launch rviz_tutorial:=true



UR5 und MoveIt mit Gripper

=====>>>>
    $1 roslaunch ur5_gripper_moveit_config demo_gazebo.launch

    Anleitung: https://roboticscasual.com/ros-tutorial-how-to-create-a-moveit-config-for-the-ur5-and-a-gripper/

in demo_gazebo.launch
    => gazebo.launch    ur5_gripper_moveit_config)/launch/gazebo.launch
        <include file="$(find ur5_gripper_moveit_config)/launch/ros_controllers.launch"/>
            => ros_controllers.launch  
                    <rosparam file="$(find ur5_gripper_moveit_config)/config/ros_controllers.yaml" command="load"/>
                  <!-- Load the controllers -->
                <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
                output="screen" args="ur5_arm_controller gripper_controller joint_state_controller"/>

                => ros_controllers.launch
                                ur5_arm_controller:
                                type: effort_controllers/JointTrajectoryController
                                joints:
                                    - shoulder_pan_joint
                                    - shoulder_lift_joint
                                    - elbow_joint
                                    - wrist_1_joint
                                    - wrist_2_joint
                                    - wrist_3_joint


            Loading controller: ur5_arm_controller
            [ERROR] [1644859873.287498326, 0.378000000]: Could not find joint 'shoulder_pan_joint' in 'hardware_interface::EffortJointInterface'.
            [ERROR] [1644859873.287681727, 0.378000000]: Failed to initialize the controller
            [ERROR] [1644859873.287717732, 0.378000000]: Initializing controller 'ur5_arm_controller' failed

            LÖSUNG:
            Change Line 45  und 45 in ros_controllers.yaml  home/oj/ws_moveit/src/ur5_gripper_moveit_config

            # was type: effort_controllers/JointTrajectoryController  
            type: position_controllers/JointTrajectoryController





ggf. hilft 
:~/ws_moveit$ catkin build
              source devel/setup.bash 


