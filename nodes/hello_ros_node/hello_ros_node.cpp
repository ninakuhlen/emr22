//------------------------------------------
// Westfälische Hochschule - FB Maschinenbau
// Labor für Mikrolektronik und Robotik
// Modul Embedded Robotics -Prof. O. Just
//--------------------------------------------------------
// hello_ros_node.cpp
// Version vom 24.03.2022
//--------------------------------------------------------
// Unser erster ROS-Knoten in C++
// Usage: 
// starten mit rosrun hello_ros_node hello_ros_node
//---------------------------------------------------------
#include <ros/ros.h>

int main (int argc, char** argv){
	ros::init(argc,argv,"hello_ROS");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Hello, ROS !!");

}

		
