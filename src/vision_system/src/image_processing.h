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
    void Threshold_Demo( int, void* );


  private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber rbg_sub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher rbg_pub_;


    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_;

    sensor_msgs::ImageConstPtr rbgIn_;
    sensor_msgs::ImageConstPtr depthIn_;
    sensor_msgs::ImageConstPtr rbgIn;
    sensor_msgs::ImageConstPtr depthIn;

    cv_bridge::CvImagePtr rbgOut_;
    cv_bridge::CvImagePtr depthOut_;



    int threshold_value = 0;
    int threshold_type = 3;;
    int const max_value = 255;
    int const max_type = 4;
    int const max_BINARY_value = 255;

    cv::Mat src, src_gray, dst;

    char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
    char* trackbar_value = "Value";

};

#endif
