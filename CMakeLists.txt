cmake_minimum_required(VERSION 3.0.2)
project(emr22)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Use C++11
add_definitions(--std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  #--------------------
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  actionlib_msgs
  control_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PythonLibs 3 REQUIRED)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES emr22
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    diagnostic_msgs
    geometry_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########


## Specify additional locations of header files and C++ - Files
## Your package locations should be listed before other locations
include_directories(
  include
   ${catkin_INCLUDE_DIRS}/emr22/src
)

## Declare  C++ executables
add_executable(hello    src/hello_ros_node.cpp  )
add_executable(turtle_pub  src/turtle_publisher.cpp  )
add_executable(turtle_sub  src/turtle_subscriber.cpp )
add_executable(turtle_p1  src/p1_my_cpp_publisher_4turtlesim.cpp )
add_executable(adaim_pick_place  src/adaim_pick_and_place.cpp )
add_executable(adaim_pick_place_collision  src/adaim_pick_and_place_collision.cpp )
add_executable(adaim_pick_place_depth  src/adaim_pick_and_place_collision_depth_camera.cpp )
add_executable(hello_ros_param    src/hello_ros_param.cpp )


#add_executable(hello_qt    src/sw01_qt_hello.cpp )
## Specify libraries to link a library or executable target against
target_link_libraries(hello ${catkin_LIBRARIES} )
target_link_libraries(turtle_pub ${catkin_LIBRARIES} )
target_link_libraries(turtle_sub ${catkin_LIBRARIES} )
target_link_libraries(turtle_p1 ${catkin_LIBRARIES} )
target_link_libraries(adaim_pick_place ${catkin_LIBRARIES} )
target_link_libraries(adaim_pick_place_collision ${catkin_LIBRARIES} )
target_link_libraries(adaim_pick_place_depth ${catkin_LIBRARIES} )
target_link_libraries(hello_ros_param ${catkin_LIBRARIES} )
# 
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Ort wo die Python-Skripte fuer die ROS-Knoten liegen
file(GLOB python_nodes_DIR
    "nodes/*/*.py"
)
# Ort wo die Python-Skripte (ohne ROS.Bezug) liegen
file(GLOB python_scripts_DIR
    "scripts/*/*.py"
)
catkin_install_python(PROGRAMS 
		       ${python_scripts_DIR}
                      ${python_nodes_DIR}
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


