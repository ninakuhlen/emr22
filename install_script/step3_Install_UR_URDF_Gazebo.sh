#### Install the needed URDF for the UR5-Robot and Gazebo ###

# Install_UR_URDF_Gazebo.sh
# ------------------------------------------------------------------
# Installing 
# auf einem Rechner mit Ubuntu 20.04 Focal Fossa  
# OJ fuer robotik.bocholt@w-hs.de
# SS2023
# geaendert am 30.3.2023
# vgl. https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html 

echo -e "\033[34m ----- robotik.bocholt@w-hs.de -- SS23 - UR5 für Gazebo installieren.. ----- \033[0m "
# echo -e "\033[34m ----- ..und Workspace ws_moveit (catkin build) einrichten ---------- \033[0m "
sudo apt-get-install ros-noetic-libuvc-ros

cd ~/ws_moveit/src
git clone https://github.com/ProfJust/universal_robot.git -b melodic-devel
# Laut Issue #537 https://github.com/ros-industrial/universal_robot/issues/537 soll diese Paket auch für ROS-Noetic funktionieren. 

# Gripper Robotiq
git clone https://github.com/ProfJust/robotiq.git -b gazebo


# --- fertige MoveIt Config aus dem Assitenten --
git clone https://github.com/ProfJust/ur5_gripper_moveit_config.git -b gazebo

# --- ros_control --- 
# UR5 Package
git clone https://github.com/dairal/ur5-joint-position-control.git
# ROS Pakete, Regler für ros_control 
sudo apt-get install ros-noetic-libuvc-ros
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers 

# ROS USB-Camera-Driver (real camera at your PC)
git clone https://github.com/ros-drivers/libuvc_ros.git
# starting 
# get camera vendor $ lsusb -v
# start $ sudo -E rosrun libuvc_camera camera_node vendor:=...

sudo apt-get install ros-noetic-rqt-joint-trajectory-controller
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

sudo apt update
sudo apt dist-upgrade

cd ~/ws_moveit
catkin build

echo -e "\033[34m --- Spawn UR5 with Gripper & MoveIt in Gazebo :  \033[0m "
echo -e "\033[34m --- roslaunch ur5_gripper_moveit_config demo_gazebo.launch --- \033[0m "

source ~/ws_moveit/devel/setup.bash
