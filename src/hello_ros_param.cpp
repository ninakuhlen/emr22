//------------------------------------------
// Westfälische Hochschule - FB Maschinenbau
// Labor für Mikrolektronik und Robotik
// Modul Embedded Robotics SS22 - Prof. Dr. O. Just
//------------------------------------------
// hello_ros_param.cpp
// Version vom 9.03.2023
//------------------------------------------
// Demo für das getten und setten von ROS-Parametern
// siehe https://roboticsbackend.com/get-set-ros-params-rospy-roscpp/
//------------------------------------------
// Kompiler: $ catkin build emr22
// => CMakeLists.txt
// add_executable(hello_ros_param    src/hello_ros_param.cpp  )
// target_link_libraries(hello_ros_param ${catkin_LIBRARIES} )
// -------------------------------------------
// usage:  
// $1 roscore
// $2 rosrun emr22 hello_ros_param 
//------------------------------------------
#include <ros/ros.h>
//---------------------------------------------
int main (int argc, char **argv){
	// ROS initialisieren , Weitergabe der Startparameter
	ros::init(argc, argv,"hello_ros_param");
	std::string string_var;
    int int_var;
   
    printf("\n Demo für das getten und setten von ROS-Parametern \n");
	//ros::param::set("/integer_param", 5);

    // alternativ auch setzen über Shell $ rosparam set integer_param 7
    // oder Launch-File
    // <launch>
    //  <param name="my_integer" type="int" value="7" />
    //  <node name="node_name" pkg="your_package" type="test_params.py" output="screen"/>
    // </launch>
    if (ros::param::has("/rosdistro")) {
        ros::param::get("/rosdistro", string_var);
        ROS_INFO(" ROS-Distro: %s", string_var.c_str()); 
    }
    if (ros::param::has("/integer_param")) {
        ros::param::get("/integer_param", int_var);
        ROS_INFO(" Ein Integer Parameter: %d", int_var); 
    }
}
