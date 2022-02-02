#### Install the neede URDF for the UR5-Robot and Gazebo ###

# Install_UR_URDF_Gazebo.sh
# ------------------------------------------------------------------
# Installing 
# auf einem Rechner mit Ubuntu 20.04 Focal Fossa  
# OJ fuer robotik.bocholt@w-hs.de
# SS2022
# geaendert am 2.2.2022
# vgl. https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html 

echo -e "\033[34m ----- robotik.bocholt@w-hs.de -- SS22 - UR5 für Gazebo installieren.. ----- \033[0m "
# echo -e "\033[34m ----- ..und Workspace ws_moveit (catkin build) einrichten ---------- \033[0m "

cd ~/ws_moveit/src
git clone https://github.com/ros-industrial/universal_robot.git -b melodic-devel
# Laut Issue #537 https://github.com/ros-industrial/universal_robot/issues/537 soll diese Paket auch für ROS-Noetic funktionieren. 

# --- ros_control --- 
# UR5 Package
git clone https://github.com/dairal/ur5-joint-position-control.git
# ROS Pakete, Regler für ros_control 
sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers

rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

sudo apt update
sudo apt dist-upgrade

cd ~/ws_moveit
catkin build

echo -e "\033[34m --- Spawn UR5 in Gazebo:  \033[0m "
echo -e "\033[34m " rosrun gazebo_ros spawn_model -file ~/ws_moveit/src/emr22/urdf/ur5_gazebo.urdf -urdf -x 0 -y 0 -z 0.1 -model ur5 ----- \033[0m "