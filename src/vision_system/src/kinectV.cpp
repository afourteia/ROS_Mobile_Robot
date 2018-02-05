#include <ros/ros.h>
#include <ros/console.h>
#include "image_processing.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinectV");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate ("~");
  ImageConverter* ic = 0;
  ic = new ImageConverter (nh, nhPrivate);

  ic->spin();
  return 0;
}
