#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include "controller_box/UKARTparams.h"
#include "controller_box/velocity.h"
#include "robot_info.h"


int main(int argc, char **argv){

	//Initialize ROS node
	ros::init(argc,argv,"robot_hub_node");		// Node name
	MICA mica;

	ros::Rate rate(10);

	while(ros::ok()){

    mica.processNode();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
