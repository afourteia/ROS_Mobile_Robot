#include <ros/ros.h>
#include <ros/console.h>
#include "image_processing.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinectV");
  ImageConverter ic;

  ic.spin();
  return 0;
}
