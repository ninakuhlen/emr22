#### Install the neede URDF for the UR5-Robot and Gazebo ###

# Install_UR5_Gripper_MoveIt _Config_4_Gazebo.sh
# ------------------------------------------------------------------
# Installing 
# auf einem Rechner mit Ubuntu 20.04 Focal Fossa  
# OJ fuer robotik.bocholt@w-hs.de
# SS2022
# geaendert am 16.2.2022
# vgl. https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html 

echo -e "\033[34m ----- robotik.bocholt@w-hs.de -- SS22 - UR5 für Gazebo installieren.. ----- \033[0m "
# echo -e "\033[34m ----- ..und Workspace ws_moveit (catkin build) einrichten ---------- \033[0m "

cd ~/ws_moveit/src
git clone https://github.com/ProfJust/universal_robot.git -b melodic-devel
# Laut Issue #537 https://github.com/ros-industrial/universal_robot/issues/537 soll diese Paket auch für ROS-Noetic funktionieren. 

# Gripper Robotiq
git clone https://github.com/filesmuggler/robotiq.git


# --- fertige MoveIt Config aus dem Assitenten --
git clone https://github.com/dairal/ur5_gripper_moveit_config

# --- ros_control --- 
sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers
# UR5 Package
git clone https://github.com/dairal/ur5-joint-position-control.git
# depth camera
git clone  https://github.com/ProfJust/common-sensors.git


# Gripper hält Box in Gazebo fest:
git clone https://github.com/PickNikRobotics/graph_msgs.git
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git -b noetic
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
git clone https://github.com/PickNikRobotics/rosparam_shortcuts.git -b noetic-devel


# fehlende ROS Pakete installieren
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

sudo apt update
sudo apt dist-upgrade

cd ~/ws_moveit
catkin build

echo -e "\033[34m --- Spawn UR5 with Gripper & MoveIt in Gazebo :  \033[0m "
echo -e "\033[34m --- roslaunch ur5_gripper_moveit_config demo_gazebo.launch --- \033[0m "

source ~/ws_moveit/devel/setup.bash
