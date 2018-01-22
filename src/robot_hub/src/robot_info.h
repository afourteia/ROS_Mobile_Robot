#ifndef __ROBOT_INFO_H_INCLUDED
#define __ROBOT_INFO_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/console.h>
#include <std_msgs.h>
// #include <std_msgs/Int8.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Int16.h>
#include <controller_box/UKARTparams.h>
#include <sensor_msgs/Joy.h>



class MICA{
  public:
    MICA();


    void cbUkartInfo(const controller_box::UKARTparams::ConstPtr& ukartInfoIncoming);

    void cbGuiInfo(const );

    void cbXboxController(const sensor_msgs::Joy:ConstPtr& joy);

    void cbMcuInfo();

    void loadMCUMessag();

    void processNode();

  private:

    controller_box::UKARTparams ukartinfo;
    std_msgs::FLoat32MultiArray mcuinfo;


    ros::NodeHandle nh;

    ros::Subscriber uKartSub;

    ros::Subscriber guiSub;
    ros::Subscriber xboxController;

    ros::Subscriber mcuSub;

    ros::Subscriber iphoneSub;

    ros::Publisher guiPub;
    ros::Publisher velocityPub;
    ros::Publisher beepPub;
    ros::Publisher calibratePub;
    ros::Publisher releaseMotorPub;
    ros::Publisher clearErrorPub;
    ros::Publisher changeControlModePub;
    ros::Publisher mcuPub;

    int velScale = 3000;



};

#endif
