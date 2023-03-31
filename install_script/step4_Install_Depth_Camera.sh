#### Install the neede URDF for the UR5-Robot and Gazebo ###

# Install_UR5_Gripper_MoveIt _Config_4_Gazebo.sh
# ------------------------------------------------------------------
# Installing 
# auf einem Rechner mit Ubuntu 20.04 Focal Fossa  
# OJ fuer robotik.bocholt@w-hs.de
# SS2023
# geaendert am 30.3.2023
# vgl. https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html 

echo -e "\033[34m ----- robotik.bocholt@w-hs.de -- SS23 - UR5 für Gazebo installieren.. ----- \033[0m "
# echo -e "\033[34m ----- ..und Workspace ws_moveit (catkin build) einrichten ---------- \033[0m "

cd ~/ws_moveit/src
g# already installed with moveIt it clone https://github.com/ProfJust/universal_robot.git -b gazebo
# Laut Issue #537 https://github.com/ros-industrial/universal_robot/issues/537 soll diese Paket auch für ROS-Noetic funktionieren. 

# Gripper Robotiq
# already installed with moveIt git clone https://github.com/ProfJust/robotiq.git -b gazebo


# --- fertige MoveIt Config aus dem Assitenten --
https://github.com/ProfJust/ur5_gripper_moveit_config.git -b gazebo

# --- ros_control --- 
sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers
# UR5 Package
git clone https://github.com/dairal/ur5-joint-position-control.git
# depth camera
git clone https://github.com/ProfJust/common-sensors.git
git clone https://github.com/ros-drivers/openni_camera.git


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
