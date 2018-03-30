#ifndef __ROBOT_INFO_H_INCLUDED
#define __ROBOT_INFO_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include "controller_box/UKARTparams.h"
#include "controller_box/velocity.h"
#include "MICA_message_package/iphone.h"

class MICA{
  public:
    MICA();


    void cbUkartInfo(const controller_box::UKARTparams& ukartInfoIncoming);

    void cbGuiInfo(const std_msgs::Float32MultiArray& mcuIncoming);

    void cbXboxController(const sensor_msgs::Joy::ConstPtr& joy);

    void cbMcuInfo(const std_msgs::Int8& mcuInfoIncoming);

    void cbiphonecmd(const MICA_message_package::iphone& iphoneIncoming);

    void cbfollowcmd(const geometry_msgs::Vector3& followIncoming);

    void processNode();

  private:

    controller_box::UKARTparams ukartinfo;
    std_msgs::Float32MultiArray mcuinfo;
    geometry_msgs::Vector3 target;

    controller_box::velocity vel;

    std_msgs::Int32MultiArray moduleStatus;


    ros::NodeHandle nh;

    ros::Subscriber uKartSub;

    ros::Subscriber guiSub;
    ros::Subscriber xboxController;

    ros::Subscriber mcuSub;

    ros::Subscriber iphoneSub;
    ros::Subscriber followSub;

    ros::Publisher guiPub;
    ros::Publisher velocityPub;
    ros::Publisher beepPub;
    ros::Publisher calibratePub;
    ros::Publisher releaseMotorPub;
    ros::Publisher clearErrorPub;
    ros::Publisher changeControlModePub;
    ros::Publisher mcuPub;

    ros::Publisher iphonePub;

    static const int velScale = 3000;
    static constexpr float depth_goal = 1.5;
    static constexpr float horiz_goal = 0;    // center of the image
    static constexpr float min_error_depth = 0.2;
    static constexpr float min_error_horiz = 0.3;

    static constexpr float DKP = 4.2;
    static constexpr float HKP = 0.05;


    int follow_state;

    float  depth_error;
    float  horiz_error;




};

#endif
