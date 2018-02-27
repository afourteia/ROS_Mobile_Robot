#include "robot_info.h"
#include <controller_box/velocity.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <MICA_message_package/iphone.h>

MICA::MICA(){

  uKartSub = nh.subscribe("ukart_parameters",1,&MICA::cbUkartInfo, this);		// Subscribe to "kart_velocity" topic
	guiSub = nh.subscribe("gui_commands",1,&MICA::cbGuiInfo, this);		// Subscribe to "beep_command"
	xboxController = nh.subscribe("joy",1,&MICA::cbXboxController, this); 	// Subscribe to "IMU_calibrate_command"
	mcuSub = nh.subscribe("mcu_info",1,&MICA::cbMcuInfo, this); 	// Subscribe to "motor_rls_command"

  // iphoneSub = nh.subscribe("iphone_control",1,&MICA::cbiphonecmd, this);
  iphoneSub = nh.subscribe("test",1,&MICA::cbiphonecmd, this);
  followSub = nh.subscribe("target_location",1,&MICA::cbfollowcmd, this);


  //guiPub = nh.advertise<controller_box::UKARTparams>("robot_info",1);
  velocityPub = nh.advertise<controller_box::velocity>("kart_velocity",1);
  beepPub = nh.advertise<std_msgs::Int8>("beep_command",1);
  calibratePub = nh.advertise<std_msgs::Int8>("IMU_calibrate_command",1);
  releaseMotorPub = nh.advertise<std_msgs::Int8>("motor_rls_command",1);
  clearErrorPub = nh.advertise<std_msgs::Int8>("clear_error_command",1);
  changeControlModePub = nh.advertise<std_msgs::Int8>("Wire_control_mode",1);
  //mcuPub = nh.advertise<std_msgs::Int8>("Wire_control_mode",1);

  //iphonePub = nh.advertise<geometry_msgs::Twist>("test",1);

  iphonePub = nh.advertise<MICA_message_package::iphone>("test",1);

  follow_state = 0;

  moduleStatus.layout.dim.push_back(std_msgs::MultiArrayDimension());
  moduleStatus.layout.dim[0].label = "data";
  moduleStatus.layout.dim[0].size = 5;
  moduleStatus.layout.dim[0].stride = 5;
}

void MICA::cbUkartInfo(const controller_box::UKARTparams& ukartInfoIncoming)
{
  ukartinfo = ukartInfoIncoming;
}

void MICA::cbGuiInfo(const std_msgs::Float32MultiArray& mcuIncoming)
{

}

void MICA::cbXboxController(const sensor_msgs::Joy::ConstPtr& joy)
{


  std_msgs::Int8 msg;
  follow_state = joy->buttons[10]; // right button stick
  if (!follow_state)
  {
    vel.linear = static_cast<int>(velScale * joy->axes[1]); // Up/Down Axis stick left
    vel.angular = static_cast<int>(velScale * joy->axes[3]); // Left/right Axis stick right
    velocityPub.publish(vel);
  }


  msg.data = joy->buttons[9]; // left button stick
  beepPub.publish(msg);

  msg.data = joy->buttons[4] + joy->buttons[5]; // LB & RB buttons
  releaseMotorPub.publish(msg);

  msg.data = joy->buttons[7]; // start button
  calibratePub.publish(msg);

  msg.data = joy->buttons[6]; // back button
  clearErrorPub.publish(msg);

  msg.data = static_cast<int>(joy->axes[7]); // cross key up/down
  changeControlModePub.publish(msg);


}

void MICA::cbMcuInfo(const std_msgs::Int8& mcuInfoIncoming)
{

}

void MICA::cbiphonecmd(const MICA_message_package::iphone& iphoneIncoming){
  moduleStatus.data.clear();
  moduleStatus.data.push_back(iphoneIncoming.x);
  moduleStatus.data.push_back(iphoneIncoming.y);
  moduleStatus.data.push_back(iphoneIncoming.z);
  moduleStatus.data.push_back(iphoneIncoming.h);
  moduleStatus.data.push_back(iphoneIncoming.m);

  mcuPub.publish(moduleStatus);
}

void MICA::cbfollowcmd(const geometry_msgs::Vector3& followIncoming)
{
  target = followIncoming;
  if (follow_state){
    depth_error = (target.z - depth_goal)/100.0;
    horiz_error = target.x - horiz_goal;

    if(fabs(depth_error) < min_error_depth && fabs(horiz_error) < min_error_horiz )
    {
      vel.linear = 0;
      vel.angular = 0;
    }else
    {
      vel.linear = DKP * depth_error * velScale ;
      vel.angular = HKP * horiz_error * velScale ;
    }

    velocityPub.publish(vel);
  }
}

void MICA::processNode()
{


}
