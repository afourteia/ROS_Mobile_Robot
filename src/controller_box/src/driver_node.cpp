#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include "kart_serial.h"
#include "controller_box/UKARTparams.h"
#include "controller_box/velocity.h"

//global variables
int linVelcmd = 0;
int angVelcmd = 0;
int beepcmd = 0;
int imuCalibcmd = 0;
int clrErrorcmd = 0;
int rlsmotorcmd = 0;
int cntlrModecmd = 0;
int sensorcalib = 0;
uint8_t publishFlag = 0;

//global kart object
UKART kart;

// Velocity commands callback function
void cbmotorCommands(const controller_box::velocity& vel){
	linVelcmd = vel.linear;
	angVelcmd = vel.angular;
	ROS_INFO_STREAM("I See Linear " << linVelcmd << "& Angular " <<  angVelcmd);
}

// Beep commands callback function
void cbbeepCommands(const std_msgs::Int8& msg){
	beepcmd = msg.data;
}

// IMU calibration callback function
void cbcalibrateCommands(const std_msgs::Int8& msg){
	imuCalibcmd = msg.data;
}

// Clear error callback function
void cbclearerrorCommands(const std_msgs::Int8& msg){
	clrErrorcmd = msg.data;
}

// release motor callback function
void cbmotorreleaseCommands(const std_msgs::Int8& msg){
	rlsmotorcmd = msg.data;
}

void cbWireControlModeCommands(const std_msgs::Int8& msg){
	cntlrModecmd = msg.data;
}
void loadROSKARTmessage(controller_box::UKARTparams& ukartinfo){

	ukartinfo.ctlMode = kart.ctlmodeACK;

	ukartinfo.mtrRPM.L =  kart.mtrRPML;
	ukartinfo.mtrRPM.R =  kart.mtrRPMR;

	ukartinfo.current.L = kart.currentL;
	ukartinfo.current.R = kart.currentR;

	ukartinfo.pitch = kart.pitch;
	ukartinfo.roll = kart.roll;

	ukartinfo.temp.L = kart.tempL;
	ukartinfo.temp.R = kart.tempR;

	ukartinfo.mtrRPMgoal.L = kart.mtrRPMgoalL;
	ukartinfo.mtrRPMgoal.R = kart.mtrRPMgoalR;

	ukartinfo.yaw = kart.yaw;
	ukartinfo.voltage = kart.voltage;

	ukartinfo.powerOFF = kart.powerOFF;

	ukartinfo.odom = kart.odom;

	ukartinfo.version = kart.version;

	ukartinfo.chipID = kart.chipID;

	ukartinfo.error = kart.error;

	ukartinfo.imuCalibAck = kart.imuCalibAck;

	ukartinfo.serCondition = kart.isConnected();

}

int main(int argc, char **argv){

	//Initialize ROS node
	ros::init(argc,argv,"driver_node");		// Node name
	ros::NodeHandle nh;
	ros::Subscriber velSub = nh.subscribe("kart_velocity",1,&cbmotorCommands);		// Subscribe to "kart_velocity" topic
	ros::Subscriber beepSub = nh.subscribe("beep_command",1,&cbbeepCommands);		// Subscribe to "beep_command"
	ros::Subscriber calibrateSub = nh.subscribe("IMU_calibrate_command",1,&cbcalibrateCommands); 	// Subscribe to "IMU_calibrate_command"
	ros::Subscriber releaseMotorSub = nh.subscribe("motor_rls_command",1,&cbmotorreleaseCommands); 	// Subscribe to "motor_rls_command"
	ros::Subscriber clearErrorSub = nh.subscribe("clear_error_command",1,&cbclearerrorCommands); 	// Subscribe to "clear_error_command"

	ros::Subscriber changeControlModeSub = nh.subscribe("Wire_control_mode",1,&cbWireControlModeCommands); 	// Subscribe to "clear_error_command"

	ros::Publisher UKARTpub = nh.advertise<controller_box::UKARTparams>("Ukart_parameters",1);	// Publish to "Ukart_parameters"
	//ros::Publisher UKARTdiagPub = nh.advertise<controller_box::UKARTdiag>("UKart_Info",1);		// Publish to "UKart_Info"


	ros::Rate rate(30); // Contoller box sends at 40hz


	controller_box::UKARTparams ukartinfo;
	//controller_box::UKARTdiag ukartInfo;

	while(ros::ok()){

		//Set the velocity commands
		kart.setVelocity(linVelcmd,angVelcmd);
		// Check for other commands
		kart.beep(beepcmd);
		kart.releaseMotor(rlsmotorcmd); //this should be called after kart.beep()

		kart.calibrateIMU(imuCalibcmd);
		kart.clearError(clrErrorcmd);

		kart.changeControlMode(cntlrModecmd);



		//Set the parity bit
		kart.send();

		publishFlag = kart.checkReceivedData();

		if(!(publishFlag == 0xFF)){
			loadROSKARTmessage(ukartinfo);
			//ukartinfo.header.stamp = ros::Time::now();
			//msg.header.frace_id = "/world";
			UKARTpub.publish(ukartinfo);
		}


		//voltPub.publish(voltagePubValue);

		ros::spinOnce();
		//rate.sleep();
	}

	return 0;
}
