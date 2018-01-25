#ifndef __IMAGE_PROCESSING_H_INCLUDED
#define __IMAGE_PROCESSING_H_INCLUDED

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodinngs.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>
#include <std_msgs/UInt8.h>

// macros
#define STOP 0
#define FOLLOW 1
#define PIX2RADS 0.00159534

// Global variables

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter{
  protected:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  public:

    ImageConverter() : it_(nh_);

    ~ImageConverter();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};
#endif
