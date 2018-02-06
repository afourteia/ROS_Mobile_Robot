#ifndef __IMAGE_PROCESSING_H_INCLUDED
#define __IMAGE_PROCESSING_H_INCLUDED

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <geometry_msgs/Vector3Stamped.h>
#include <math.h>
#include <std_msgs/UInt8.h>
#include <stdlib.h>
#include <stdio.h>

class ImageConverter
{

  public:

    ImageConverter();
    ~ImageConverter();
    void rbgCb(const sensor_msgs::ImageConstPtr& img);
    void depthCb(const sensor_msgs::ImageConstPtr& img);
    void spin();
    void process();

  protected:
    image_transport::ImageTransport it_;
    image_transport::Subscriber rbg_sub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher image_pub_;


    ros::NodeHandle nh_;

    sensor_msgs::ImageConstPtr rbgIn_;
    sensor_msgs::ImageConstPtr depthIn_;
    sensor_msgs::ImageConstPtr rbgIn;
    sensor_msgs::ImageConstPtr depthIn;

    cv_bridge::CvImagePtr rbgOut_;
    cv_bridge::CvImagePtr depthOut_;

    const std::string RGB_WINDOW = "rgb window";

};

#endif
