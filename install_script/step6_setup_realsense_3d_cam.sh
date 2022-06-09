# step6_setup_realsense_3_cam.sh
# ------------------------------------------------------------------
# Installing Intel Realsense 3D-Cam on ROS Noetic for use with MoveIt!
# auf einem Rechner mit Ubuntu 20.04 Focal Fossa  
# OJ fuer robotik.bocholt@w-hs.de
# SS2022
# geaendert am 9.6.2022
#------------------------------------------------------------------
# funktioniert nicht aber gute Erklärung =>
# https://linuxtut.com/en/be1eb78015828d43f9fb/
#------------------------------------------------------------------
#  usage:
# $ roslaunch realsense2_camera rs_camera.launch
# $ roslaunch emr22 find_2d_object_realsense_world
# ggf.   rviz
# rosrun tf tf_echo world object_14
#------------------------------------------------------------------

echo -e "\033[34m ----- robotik.bocholt@w-hs.de -- SS22 - Intel Realsense installieren .. ----- \033[0m "

cd ~/ws_moveit
cd ~/ws_moveit/src/
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-db

sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-object-recognition-msgs
sudo apt-get install ros-noetic-libuvc-camera
sudo apt-get install ros-noetic-octomap-msgs
sudo apt-get install ros-noetic-eigen-stl-containers
sudo apt-get install ros-noetic-random-numbers
sudo apt-get install ros-noetic-ddynamic_reconfigure
sudo apt-get install ros-noetic-ompl

echo -e "\033[34m Intel Realsense SDK-Tool starten mit: $ realsense-viewer \033[0m "

cd ~/ws_moveit/src/
# ROS-Wrapper
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/ros-planning/warehouse_ros.git

