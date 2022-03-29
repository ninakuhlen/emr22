//------------------------------------------
// Westfälische Hochschule - FB Maschinenbau
// Labor für Mikrolektronik und Robotik
// Modul Embedded Robotics SS22 - Prof. Dr. O. Just
//------------------------------------------
// p1_my_cpp_publisher_4turtlesim
// Version vom 25.03.2022
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
	ros::Rate rate(2); // Senderate 2Hz

	srand(time(0)); //Zufallsgenerator seeden
	//printf("\n Nutzen Sie wasd ein, um die Turtle zu steuern, q = quit \n");

	while(ros::ok()){  //Endlosschleife bis rosshutdown
		// Instanziere Message
		geometry_msgs:: Twist myMsg;
		
		myMsg.linear.x  =     double(rand())/double(RAND_MAX);	
		myMsg.angular.z = 2 * double(rand())/double(RAND_MAX);

		// senden der msg
		myPub.publish(myMsg);		
		// Warte bis zur nächsten Sendung
		rate.sleep();
	}
}

//-----------------------------------------------------------
// Funktion zum holen eine Taste ohne Enter 
// https://stackoverflow.com/questions/7469139/what-is-the-equivalent-to-getch-getche-in-linux
// ggf. ncurses.h nutzen #include <ncurses.h>  c = getch();
// siehe https://stackoverflow.com/questions/905060/non-blocking-getch-ncurses
//-----------------------------------------------------------
char getch(void){
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