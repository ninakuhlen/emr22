#### Install MoveIt! and Tutorial on ROS 1 ###

# Install_MoveIt_on_ROS1.sh
# ------------------------------------------------------------------
# Installing MoveIt! - Noetic 
# auf einem Rechner mit Ubuntu 20.04 Focal Fossa  
# OJ fuer robotik.bocholt@w-hs.de
# SS2022
# geaendert am 27.1.2022
# vgl. https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html 

echo -e "\033[34m ----- robotik.bocholt@w-hs.de -- SS22 - MoveIt! installieren.. ----- \033[0m "
echo -e "\033[34m ----- ..und Workspace ws_moveit (catkin build) einrichten ---------- \033[0m "

cd ~/ws_moveit

# rosdep update
# sudo apt update
# sudo apt dist-upgrade

# Install catkin the ROS build system:

sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon
sudo apt install python3-wstool


# MoveIt! verwendet catkin build statt catkin_make, daher muss hier ein eigener Workspace her
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src

# wstool provides commands to manage several local SCM repositories 
#(supports git, mercurial, subversion, bazaar) based on a single workspace definition file (.rosinstall). 
wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove  moveit_tutorials  # this is cloned in the next section
wstool update -t .

git clone https://github.com/ros-planning/moveit_tutorials.git -b master
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel

rosdep install -y --from-paths . --ignore-src --rosdistro noetic

# ggf. vorher Moveit mit APT installiert => sudo apt purge "ros-${ROS_DISTRO}-moveit-*" 
cd ~/ws_moveit
#catkin_make war früher => Dateien im Ordner devel und build löschen
catkin clean
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
