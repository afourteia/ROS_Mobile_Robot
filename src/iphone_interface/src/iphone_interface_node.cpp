#include "ros/ros.h"
#include "std_msgs/String.h"

// call back function for iphone commands
void iphonecmdCb(){

}

int main(int argc, char **argv){
  // Initialize ros and ros node
  ros::init(argc, argv, "iphone_interface_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = n.subscribe("iphone_control", 1,iphonecmdCb);
  ros::Publisher pub = n.advertise</*messagetype*/>("/*cmdtopic*/");

  while(ros::ok()){
      ros::spin();
  }

  return 0;
}
