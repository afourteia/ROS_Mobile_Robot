#ifndef __ROBOT_INFO_H_INCLUDED
#define __ROBOT_INFO_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/console.h>
#include <std_msgs.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <controller_box/UKARTparams.h>

class MICA{
  public:
    MICA();

    ros::NodeHandle nh;

    
    ros::Subscriber uKartSub;

    ros::Subscriber guiSub;
    ros::Publisher guiPub;

    ros::Publisher velocityPub;
    ros::Publisher beepPub;
    ros::Publisher calibratePub;
    ros::Publisher releaseMotorPub;
    ros::Publisher clearErrorPub;
    ros::Publisher changeControlModePub;
    ros::Publisher mcuPub;





  private:

};

#endif
