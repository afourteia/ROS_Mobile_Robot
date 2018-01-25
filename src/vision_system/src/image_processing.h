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


class ImageConverter
{

  public:

    ImageConverter();
    ~ImageConverter();
    void rbgCb(const sensor_msgs::ImageConstPtr& img);
    void depthCb(const sensor_msgs::ImageConstPtr& img);
    void spin();
    void process();


  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber rbg_sub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher image_pub_;


    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_;

    sensor_msgs::ImageConstPtr rbgIn_;
    sensor_msgs::ImageConstPtr depthIn_;
    cv_bridge::CvImage imageOut_;

    static const std::string OPENCV_WINDOW = "Image window";
};

#endif
