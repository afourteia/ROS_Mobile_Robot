#include <ros/ros.h>
#include <ros/console.h>
#include "image_processing.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  //ros::NodeHandle nh;
  //ros::NodeHandle nhPrivate ("~");
  ImageConverter ic;

  ic.spin();
  return 0;
}
