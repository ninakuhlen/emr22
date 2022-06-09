# step6_setup_realsense_3_cam.sh
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

cd ~/ws_moveit/src/
590  sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  591  sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
  592  sudo apt-get install librealsense2-dkms
  593  sudo apt-get install librealsense2-utils
  594  sudo apt-get install librealsense2-dev
  595  sudo apt-get install librealsense2-dbg
  596  realsense-viewer
  597  history



ROS-Wrapper
https://github.com/IntelRealSense/realsense-ros.git

sudo apt-get install ros-noetic-realsense2-camera
sudo apt-get install ros-noetic-object-recognition-msgs
sudo apt-get install ros-noetic-libuvc-camera
sudo apt-get install ros-noetic-octomap-msgs
sudo apt-get install ros-noetic-eigen-stl-containers
sudo apt-get install ros-noetic-random-numbers
sudo apt-get install ros-noetic-ddynamic_reconfigure


funktioniert nicht aber gute Erklärung =>
https://linuxtut.com/en/be1eb78015828d43f9fb/



usage:


roslaunch realsense2_camera rs_camera.launch

roslaunch emr22 find_2d_object_realsense_world

ggf.   rviz

rosrun tf tf_echo world object_14
