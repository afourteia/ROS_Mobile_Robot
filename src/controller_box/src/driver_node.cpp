#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt16.h>
#include "kart_serial.h"
#include ""


//global variables
int linVelcmd = 0;
int angVelcmd = 0;
int beepcmd = 0;
int imuCalibcmd = 0;
int clrErrorcmd = 0;
int rlsmotorcmd = 0;
int sensorcalib = 0;
uint8_t publishFlag = 0;


// Velocity commands callback function
void callbackmotorCommands(const geometry_msgs::Twist& vel){
	linVelcmd = (int)(vel.linear.x);
	angVelcmd = (int)(vel.angular.z * kart.CartRadius);
}

// Beep commands callback function
void callbackbeepCommands(const std_msgs::Int8& msg){
	beepcmd = msg.data;
}

// IMU calibration callback function
void callbackcalibrateCommands(const std_msgs::Int8& msg){
	imuCalibcmd = msg.data;
}

// Clear error callback function
void callbackclearerrorCommands(const std_msgs::Int8& msg){
	clrErrorcmd = msg.data;
}

// release motor callback function
void callbackmotorreleaseCommands(const std_msgs::Int8& msg){
	rlsmotorcmd = msg.data;
}


int main(int argc, char **argv){

	//Initialize ROS node
	ros::init(argc,argv,"driver_node");		// Node name
	ros::NodeHandle nh;
	ros::Subscriber velSub = nh.subscribe("kart_velocity",1,callbackmotorCommands);		// Subscribe to "kart_velocity" topic
	ros::Subscriber beepSub = nh.subscribe("beep_command",1,callbackbeepCommands);		// Subscribe to "beep_command"
	ros::Subscriber calibrateSub = nh.subscribe("IMU_calibrate_command",1,callbackcalibrateCommands); 	// Subscribe to "IMU_calibrate_command"
	ros::Subscriber releaseMotorSub = nh.subscribe("motor_rls_command",1,callbackmotorreleaseCommands); 	// Subscribe to "motor_rls_command"
	ros::Subscriber clearErrorSub = nh.subscribe("clear_error_command",1,callbackclearerrorCommands); 	// Subscribe to "clear_error_command"
	ros::Publisher voltPub = nh.advertise<std_msgs::Float32>("batteryInfo",1);	// Publish to "batteryInfo"
	ros::Publisher errorPub = nh.advertise<std_msgs::Int8>("errorInfo",1);		// Publish to "errorInfo"
	ros::Publisher currentPub = nh.advertise<std_msgs::Float32>("currentInfo",1);	// Publish to "currentInfo"
	ros::Publisher tempPub	= nh.advertise<std_msgs::Float32>("tempInfo",1);	// Publish to "tempInfo"
	ros::Publisher SerCondPub	= nh.advertise<std_msgs::Int8>("serCondition",1);	// Publish to "serCondition"


	UKART kart;


	ros::Rate rate(10);

	std_msgs::Float32 voltagePubValue;

	while(ros::ok()){

		//Set the velocity commands
		kart.setVelocity(linVelcmd,angVelcmd);
		// Check for other commands
		kart.beep(beepcmd);

		//Set the parity bit
		kart.send();

		publishFlag = kart.checkReceivedData();

		switch (publishFlag) {
			case WCEMA:
				break;
			case WCExMA:
				break;
			case SpInfo:;
				mtrRPML
				mtrRPMR
				break;
			case CurInfo:
				currentL = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
				currentR = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
				break;
			case AttdInfo:
				pitch = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
				roll = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
				break;
			case TempInfo:
				tempL = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
				tempR = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
				break;
			case SSpInfo:
				speedlinGoal = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
				speedAngGoal = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
				break;
			case YawVoltInfo:
				yaw = (cmdrecieve[info_4]<<8) + cmdrecieve[info_3];
				voltage = (cmdrecieve[info_2]<<8) + cmdrecieve[info_1];
				break;
			case PwOFF:
				break;
			case ODOInfo:
				odom = (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16)
				 	+ (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]);
				break;
			case VerInfo:
				version = (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16)
				 	+ (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]);
				break;
			case ChipIDInfo:
				chipID = (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16)
				 	+ (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]);
				break;
			case ErrorInfo:
				error = (cmdrecieve[info_4]<<24) + (cmdrecieve[info_3]<<16)
				 	+ (cmdrecieve[info_2]<<8) + (cmdrecieve[info_1]);
				break;
			case GACA:
				return GACA;
				break;
			default:
				break;
		};

		voltPub.publish(voltagePubValue);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
