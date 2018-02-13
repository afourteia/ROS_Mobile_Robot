#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <iostream>
#include <stdio.h>
#include <vector>

static const std::string RGB_WINDOW = "rgb window";
static const std::string FILTER_WINDOW = "filtered window";
static const std::string CIRCLE_WINDOW = "circles window";
// Blue target
int HueH = 136;
int HueL = 123;
int SatH = 176;
int SatL = 68;
int ValH = 255;
int ValL = 0;

int cannyThreshold = 100;
int accumulatorThreshold = 50;
int radiusThreshold = 150;

int GaussianBlurSigma = 2;

// int HueH = 227;
// int HueL = 159;
// int SatH = 225;
// int SatL = 183;
// int ValH = 221;
// int ValL = 11;


class ImageConverter
{

protected:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber rbg_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;

  sensor_msgs::ImageConstPtr rbgIn_;
  sensor_msgs::ImageConstPtr depthIn_;

  cv_bridge::CvImagePtr rbgOut_;
  cv_bridge::CvImagePtr depthOut_;

  std::vector<cv::Vec3f> circles;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

    ROS_INFO_STREAM("Creating an IC object");
    rbg_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
      &ImageConverter::rbgCb, this);

    depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
      &ImageConverter::depthCb, this);


    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(RGB_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(FILTER_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(CIRCLE_WINDOW, CV_WINDOW_AUTOSIZE);


    cv::createTrackbar("Hue max threshold", FILTER_WINDOW, &HueH, HueH);
    cv::createTrackbar("Hue min threshold", FILTER_WINDOW, &HueL, HueH);
    cv::createTrackbar("Sat max threshold", FILTER_WINDOW, &SatH, SatH);
    cv::createTrackbar("Sat min threshold", FILTER_WINDOW, &SatL, SatH);
    cv::createTrackbar("Value max threshold", FILTER_WINDOW, &ValH, ValH);
    cv::createTrackbar("Value min threshold", FILTER_WINDOW, &ValL, ValH);

    cv::createTrackbar("Canny Threshold", CIRCLE_WINDOW, &cannyThreshold, 200);
    cv::createTrackbar("Accumulator Threshold", CIRCLE_WINDOW, &accumulatorThreshold, 200);
    cv::createTrackbar("Gaussian Kernal STD", CIRCLE_WINDOW, &GaussianBlurSigma, 10);
    cv::createTrackbar("Gaussian Kernal STD", CIRCLE_WINDOW, &radiusThreshold, 480);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
    cv::destroyWindow(FILTER_WINDOW);
  }

  void rbgCb(const sensor_msgs::ImageConstPtr& img)
  {
    ROS_INFO_STREAM("receiving an rgb image");
    if(img == NULL) ROS_INFO_STREAM("cb rbg is NULL");
    rbgIn_ = img;
  }


  void depthCb(const sensor_msgs::ImageConstPtr& img)
  {
    ROS_INFO_STREAM("receiving a depth image");
    if(img == NULL) ROS_INFO_STREAM("cb depth is NULL");
    depthIn_ = img;
  }


  void process()
  {
    // Atonomicly copy the image pointer so it doesn't change during process();
    ROS_INFO_STREAM("processing");
    sensor_msgs::ImageConstPtr rbgIn = rbgIn_;
    sensor_msgs::ImageConstPtr depthIn = depthIn_;

    ROS_INFO_STREAM("Checking for Null image");
    // Check if image if rbg image has been processed before
    if(rbgIn == NULL)// || depthIn == NULL)
    {
      ROS_INFO_STREAM("rbgIn is null");
      return;
    }

    if(depthIn == NULL)
    {
      ROS_INFO_STREAM("depthIn is null");
      return;
    }


    // ROS_INFO_STREAM("checking image sequence");
    // // Check if image if rbg image has been processed before
    // if(rbgOut_->header.seq == rbgIn->header.seq)
    //   return;

    ROS_INFO_STREAM("converting ros image to opencv image");
    try{
      rbgOut_ = cv_bridge::toCvCopy(rbgIn, sensor_msgs::image_encodings::BGR8);
    }catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    try{
      depthOut_ = cv_bridge::toCvCopy(depthIn, sensor_msgs::image_encodings::TYPE_8UC1);
    }catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ROS_INFO_STREAM("Applying filter");
    // Filtering
    cv::Mat gray;
    cv::Mat HSV;
    cv::Mat HSV_mask;
    cv::Mat HSV_filtered;
    cv::Mat BGR_filtered;
    ROS_INFO_STREAM("converting to gray");
    cv::cvtColor(rbgOut_->image, gray, CV_BGR2GRAY);
    ROS_INFO_STREAM("detecting circles");
    cv::medianBlur(gray, gray, 5);
    //cv::GaussianBlur( gray, gray, cv::Size(9, 9), GaussianBlurSigma, GaussianBlurSigma );
    cv::HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/10, cannyThreshold, accumulatorThreshold, 0, radiusThreshold );


    // for( size_t i = 0; i < circles.size(); i++ )
    // {
    //   ROS_INFO_STREAM("detecting circles");
    //    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //    int radius = cvRound(circles[i][2]);
    //    // circle center
    //    cv::circle( gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
    //    // circle outline
    //    cv::circle( gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    //
    //  }


    ROS_INFO_STREAM("converting to HSV");
    cv::cvtColor(rbgOut_->image, HSV, CV_BGR2HSV);
    ROS_INFO_STREAM("applying limits");
    cv::inRange(HSV, cv::Scalar(HueL,SatL,ValL),cv::Scalar(HueH,SatH,ValH),HSV_mask);
    ROS_INFO_STREAM("applying mask");
    HSV.copyTo(HSV_filtered,HSV_mask);
    ROS_INFO_STREAM("converting to BGR");
    cv::cvtColor(HSV_filtered, BGR_filtered, CV_HSV2BGR);

    int density = 0;
    int j = -1;
    int target_radius;
    cv::Point target_center;
    for( size_t i = 0; i < circles.size(); i++ )
    {
      ROS_INFO_STREAM("detecting circles");
       cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
       int radius = cvRound(circles[i][2]);
       ROS_INFO_STREAM("creating the rectangle");
       cv::Rect r(center.x-radius, center.y-radius, radius*2, radius*2);
      /// adding if statements here
      if((center.x-radius > 0 && center.y-radius > 0 && radius+center.x < HSV_mask.cols && center.y+radius < HSV_mask.rows))
      {
        ROS_INFO_STREAM("creating the ROI");
        cv::Mat roi(HSV_mask, r);
        ROS_INFO_STREAM("density check");
        if (density < cvRound((cv::countNonZero(roi)/(roi.cols*roi.rows))*100))
        {
          j = i;
          target_center = center;
          target_radius = radius;

        }

      }

       // circle center
       cv::circle( gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
       // circle outline
       cv::circle( gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

    }


    if(j <= 0)
    {
      ROS_INFO_STREAM("Drawing the circle");
      cv::circle( gray, target_center, target_radius, cv::Scalar(44,55,155), 10, 6, 0 );
    }


    // Update GUI Window
    ROS_INFO_STREAM("updating image window");
    cv::imshow(RGB_WINDOW, rbgOut_->image);
    cv::imshow(FILTER_WINDOW, BGR_filtered);
    cv::imshow(CIRCLE_WINDOW, gray);

    cv::waitKey(1);

    // Output modified video stream
    ROS_INFO_STREAM("publishing");
    image_pub_.publish(rbgOut_->toImageMsg());
  }

  void spin()
  {
    ROS_INFO_STREAM("spinning");
    ros::Rate rate(30);
    while(ros::ok())
    {
      process();
      ros::spinOnce();
      ROS_INFO_STREAM("sleeping");
      rate.sleep();
    }
  }

};

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("starting node");
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ic.spin();
  ROS_INFO_STREAM("stopping node");
  return 0;
}
