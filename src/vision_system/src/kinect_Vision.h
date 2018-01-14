#ifndef __KINECT_VISION_H_INCLUDED
#define __KINECT_VISION_H_INCLUDED

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


class ImageConverter{
  protected:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  public:

    ImageConverter() : it_(nh_){
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/image_raw", 1,
        &ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);

      cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter(){
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodinngs::BGR8);
      }catch()
    }
};
#endif
