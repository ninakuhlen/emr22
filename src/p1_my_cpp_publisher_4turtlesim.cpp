//------------------------------------------
// Westfälische Hochschule - FB Maschinenbau
// Labor für Mikrolektronik und Robotik
// Modul Embedded Robotics SS22 - Prof. Dr. O. Just
//------------------------------------------
// p1_my_cpp_publisher_4turtlesim
// Version vom 9.03.2023
//------------------------------------------
// ROS -Publisher für die Steuerung der
// Turtle-Bot Simulation
//------------------------------------------
// Kompiler: $ catkin build emr22
// usage:  
// $1 roscore
// $2 rosrun turtlesim turtlesim_node 
// $3 rosrun emr22 turtle_p1
//------------------------------------------
#include <ros/ros.h>

//The srv class for the service.
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>

// ROS-Klasse für den Nachrichten-Typ einbinden
#include <geometry_msgs/Twist.h>
#include <stdlib.h> //wegen rand() und srand()

// Funktion zum lesen einer Taste ohne Enter - s.u.
// getch() https://stackoverflow.com/questions/7469139/what-is-the-equivalent-to-getch-getche-in-linux
#include <unistd.h>
#include <termios.h>
char getch(void);

//---------------------------------------------
int main (int argc, char **argv){
	// ROS initialisieren , Weitergabe der Startparameter
	ros::init(argc, argv,"publish_velocity");
	// Eine eigenen Knoten instanzieren
	ros::NodeHandle myNh; 
	// Die Template-Funktion mit dem Datentyp der Parameter versehen
	ros::Publisher myPub = myNh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);  
    ros::Publisher myPub2 = myNh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1000);  

    // #### setup spawn service ###
        // $ rosservice call /spawn 2 3 0.1 "turtle2"
        // see https://sir.upc.edu/projects/rostutorials/5-client-server_tutorial/index.html
        // Create a client object for the spawn service. This
        // needs to know the data type of the service and its name.
        ros::ServiceClient spawnClient
        = myNh.serviceClient<turtlesim::Spawn>("spawn");
        //Create the request and response objects.
                       turtlesim::Spawn::Request req;
                       turtlesim::Spawn::Response resp;
                       std::string string_var;
                       bool success;
    // #### end ###

    ros::ServiceClient clearClient
        = myNh.serviceClient<std_srvs::Empty>("clear");
        std_srvs::Empty::Request req_clear;
        std_srvs::Empty::Response resp_clear;

	ros::Rate rate(2); // Senderate 2Hz

	//srand(time(0)); //Zufallsgenerator seeden
	printf("\n Nutzen Sie wasd ein, um die Turtle zu steuern, q = quit, n = spawn new turtle2, r = rosparam \n");

	while(ros::ok()){  //Endlosschleife bis rosshutdown
		// Instanziere Message
		geometry_msgs:: Twist myMsg;
		char key = getch();
		if(key >32) printf("\n got %c key  ",key);
		switch(key){
			case 'w': myMsg.linear.x  = 0.5;  break;
			case 's': myMsg.linear.x  = -0.5; break;
			case 'a': myMsg.angular.z = 0.5; break;
			case 'd': myMsg.angular.z = -0.5; break;
			case 'q': exit(1); //quit program
            case 'n': 
                      // #### call spawn service ###
                       req.x = 2;
                       req.y = 3;
                       req.theta = M_PI/2;
                       req.name = "turtle2";

                       ros::service::waitForService("spawn", ros::Duration(5));
                       success = spawnClient.call(req,resp);

                       if(success){
                          ROS_INFO_STREAM("Spawned a turtle named "
                                        << resp.name);
                       }
                       else{
                            ROS_ERROR_STREAM("Failed to spawn.");
                       }
					   // #### end call spawn service ###
                    break;
            
            case 'r': 
                // https://roboticsbackend.com/get-set-ros-params-rospy-roscpp/
                //  ros::param::set("/another_integer", 5);

                if (ros::param::has("/rosdistro")) {
                    ros::param::get("/rosdistro", string_var);
                    ROS_INFO(" ROS-Distro: %s", string_var.c_str()); 
                }
                if (ros::param::has("/rosdistro")) {
                    ros::param::get("/rosdistro", string_var);
                    ROS_INFO(" ROS-Distro: %s", string_var.c_str()); 
                }
                if (ros::param::has("turtlesim/background_b")) {
                    ros::param::set("/turtlesim/background_b", 50);
                }

                if (ros::param::has("turtlesim/background_g")) {
                    ros::param::set("/turtlesim/background_g", 50);
                }
                if (ros::param::has("turtlesim/background_r")) {
                    ros::param::set("/turtlesim/background_r", 50);
                }
                // http://wiki.ros.org/turtlesim
                // Clears the turtlesim background
                // and sets the color to the value of the background parameters. 
                ros::service::waitForService("clear", ros::Duration(5));
                success = clearClient.call(req_clear, req_clear);
                if(success){
                    ROS_INFO_STREAM("Cleared Turtlesim ");
                }
                else{
                    ROS_ERROR_STREAM("Failed ");
                }                            
            break;
         } //switch
       
		// senden der msg
		myPub.publish(myMsg);
        myPub2.publish(myMsg);		
		// Warte bis zur nächsten Sendung
		rate.sleep();
	}
}

//-----------------------------------------------------------
// Funktion zum holen eine Taste ohne Enter 
//-----------------------------------------------------------
char getch(void)
{
    char buf = 0;
    struct termios old = {0};
    fflush(stdout);
    if(tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if(tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if(read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if(tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    //printf("%c\n", buf);
    return buf;
 }
