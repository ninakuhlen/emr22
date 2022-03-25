//------------------------------------------
// Westfälische Hochschule - FB Maschinenbau
// Labor für Mikrolektronik und Robotik
// Modul Embedded Robotics -Prof. O. Just
//------------------------------------------
// turtle_publisher.cpp
// Version vom 25.03.2022
//------------------------------------------
// ROS -Publisher für die Steuerung der
// Turtle-Bot Simulation
// siehe auch:
// A Gentle Introduction to ROS von J.O.Kane
// dont forget to start turtlesim first
// rosrun turtlesim turtlesim_node 
//------------------------------------------

#include "ros/ros.h"
#include "turtlesim/Pose.h"

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {

  ROS_INFO("Position of turtle: x:[%f] y:[%f] theta:[%f]", msg->x, msg->y, msg->theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/turtle1/pose", 100, poseCallback);
    ros::spin();
    return 0;
}