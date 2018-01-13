#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include "kart_serial.h"


//global variables
int linVelcmd = 0;
int angVelcmd = 0;
int beepcmd = 0;
int rlsmotorcmd = 0;
int sensorcalib = 0;

// Velocity commands callback function
void callbackmotorCommands(const geometry_msgs::Twist& vel){
	linVelcmd = (int)(vel.linear.x);
	angVelcmd = (int)(vel.angular.z * CartRadius);
}

// Beep commands callback function
void callbackbeepCommands(const std_msgs::Int8& msg){
	beepcmd = msg.data;
}


int main(int argc, char **argv){

	//Initialize ROS components
	ros::init(argc,argv,"driver_node");		// Node name
	ros::NodeHandle nh;
	ros::Subscriber velSub = nh.subscribe("kart_velocity",1,callbackmotorCommands);		// Subscribe to "kart_velocity" topic
	ros::Subscriber beepSub = nh.subscribe("beep_command",1,callbackbeepCommands);		// Subscribe to "beep_command"
	ros::Publisher voltPub = nh.advertise<std_msgs::Float32>("batteryInfo",1);	// Publish to "batteryInfo"
	ros::Publisher errorPub = nh.advertise<std_msgs::Int8>("errorInfo",1);		// Publish to "errorInfo"
	ros::Publisher currentPub = nh.advertise<std_msgs::Float32>("currentInfo",1);	// Publish to "currentInfo"
	ros::Publisher tempPub	= nh.advertise<std_msgs::Float32>("tempInfo",1);	// Publish to "tempInfo"

	// Setup serial comm and check if valid
	if(!serialSetup(&cmdSetup[0]))	return 0;

	ros::Rate rate(10);

	while(ros::ok()){

		//Set the velocity commands
		cmdSend[LINVELL] = 0xff & linVelcmd;
		cmdSend[LINVELH] = 0xff & (linVelcmd >> 8);
		cmdSend[ANGVELL] = 0xff & angVelcmd;
		cmdSend[ANGVELH] = 0xff & (angVelcmd >> 8);

		// Check for other commands
		if(beepcmd > 0){
					cmdSend[CTL_BYTE]	= BEEP_CTL_BIT;
					beepcmd--;
		}else{
			cmdSend[CTL_BYTE] = 0;
		}

		//Set the parity bit
		cmdSend[XOR] = parityBit(&cmdSend[0], 12);

		ser.write(&cmdSend[0],13);

		checkReceivedData();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
