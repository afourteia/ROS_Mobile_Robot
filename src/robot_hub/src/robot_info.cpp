#include "robot_info.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs.h>



MICA::MICA(){

  uKartSub = nh.subscribe("ukart_parameters",1,&MICA::cbUkartInfo, this);		// Subscribe to "kart_velocity" topic
	guiSub = nh.subscribe("gui_commands",1,&MICA::cbGuiInfo, this);		// Subscribe to "beep_command"
	xboxController = nh.subscribe("joy",1,&MICA::cbXboxController, this); 	// Subscribe to "IMU_calibrate_command"
	mcuSub = nh.subscribe("mcu_info",1,&MICA::cbMcuInfo, this); 	// Subscribe to "motor_rls_command"

  iphoneSub = nh.subscribe("iphone_control",1,&MICA::cbiphonecmd, this);

  //guiPub = nh.advertise<controller_box::UKARTparams>("robot_info",1);
  velocityPub = nh.advertise<geometry_msgs::Twis>("kart_velocity",1);
  beepPub = nh.advertise<std_msgs::Int8>("beep_command",1);
  calibratePub = nh.advertise<std_msgs::Int8>("IMU_calibrate_command",1);
  releaseMotorPub = nh.advertise<std_msgs::Int8>("motor_rls_command",1);
  clearErrorPub = nh.advertise<std_msgs::Int8>("clear_error_command",1);
  changeControlModePub; = nh.advertise<std_msgs::Int8>("Wire_control_mode",1);
  //mcuPub = nh.advertise<std_msgs::Int8>("Wire_control_mode",1);
}

void MICA::cbUkartInfo(const controller_box::UKARTparams::ConstPtr& ukartInfoIncoming){
  ukartinfo = ukartInfoIncoming;
}

void MICA::cbGuiInfo(const std_msgs::FLoat32MultiArray& mcuIncoming){
  mcuinfo = ukartIncoming;
}

void MICA::cbXboxController(const sensor_msgs::Joy:ConstPtr& xboxControllerIncoming){
  
  velocityPub.publish();
  beepPub.publish();
  calibratePub.publish();
  releaseMotorPub.publish();
  clearErrorPub.publish();
  changeControlModePub.publish();
}

void MICA::cbMcuInfo(const std_msgs::Int8& mcuInfoIncoming){

}

void MICA::cbiphonecmd(const std_msgs::Int8& iphoneIncoming){

}

void processNode(){



}
