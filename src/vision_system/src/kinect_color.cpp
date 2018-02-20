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
#include <geometry_msgs/Vector3.h>


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

int morph_elem = 0;
int morph_size  = 0;

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
  ros::Publisher goal_pub;

  sensor_msgs::ImageConstPtr rbgIn_;
  sensor_msgs::ImageConstPtr depthIn_;

  cv_bridge::CvImagePtr rbgOut_;
  cv_bridge::CvImagePtr depthOut_;

  std::vector<cv::Vec3f> circles;
  geometry_msgs::Vector3 target;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

    ROS_INFO_STREAM("Creating an IC object");
    rbg_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
      &ImageConverter::rbgCb, this);

    depth_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1,
      &ImageConverter::depthCb, this);


    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    goal_pub = nh_.advertise<geometry_msgs::Vector3>("target_location", 1);

    cv::namedWindow(RGB_WINDOW, CV_WINDOW_NORMAL );
    cv::namedWindow(FILTER_WINDOW, CV_WINDOW_NORMAL);
    cv::namedWindow(CIRCLE_WINDOW, CV_WINDOW_NORMAL);


    cv::createTrackbar("Hue max threshold", FILTER_WINDOW, &HueH, 255);
    cv::createTrackbar("Hue min threshold", FILTER_WINDOW, &HueL, 255);
    cv::createTrackbar("Sat max threshold", FILTER_WINDOW, &SatH, 255);
    cv::createTrackbar("Sat min threshold", FILTER_WINDOW, &SatL, 255);
    cv::createTrackbar("Value max threshold", FILTER_WINDOW, &ValH, 255);
    cv::createTrackbar("Value min threshold", FILTER_WINDOW, &ValL, 255);

    cv::createTrackbar("Canny Threshold", CIRCLE_WINDOW, &cannyThreshold, 200);
    cv::createTrackbar("Accumulator Threshold", CIRCLE_WINDOW, &accumulatorThreshold, 200);
    cv::createTrackbar("Gaussian Kernal STD", CIRCLE_WINDOW, &GaussianBlurSigma, 10);
    cv::createTrackbar("Radius Threshold", CIRCLE_WINDOW, &radiusThreshold, 480);
    cv::createTrackbar("Morphology kernal R:0, C:0 Shape", CIRCLE_WINDOW, &morph_elem, 1);
    cv::createTrackbar("Morphology kernal size", CIRCLE_WINDOW, &morph_size, 5);
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
      depthOut_ = cv_bridge::toCvCopy(depthIn, sensor_msgs::image_encodings::TYPE_16UC1);
    }catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ROS_INFO_STREAM("Applying filter");
    // Filtering
    cv::Mat BGR;
    cv::Mat gray;
    cv::Mat HSV;
    cv::Mat HSV_mask;
    cv::Mat HSV_filtered;
    cv::Mat BGR_filtered;
    cv::Mat element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ));

    ROS_INFO_STREAM("Smothing");
    cv::medianBlur(rbgOut_->image, BGR, 5);
    //cv::GaussianBlur( gray, gray, cv::Size(9, 9), GaussianBlurSigma );

    ROS_INFO_STREAM("converting to gray");
    cv::cvtColor(BGR, gray, CV_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    //cv::GaussianBlur( gray, gray, cv::Size(9, 9), GaussianBlurSigma );



    ROS_INFO_STREAM("converting to HSV");
    cv::cvtColor(BGR, HSV, CV_BGR2HSV);
    ROS_INFO_STREAM("applying limits");
    cv::inRange(HSV, cv::Scalar(HueL,SatL,ValL),cv::Scalar(HueH,SatH,ValH),HSV_mask);
    cv::morphologyEx(HSV_mask, HSV_mask, 2,element);  //Open
    // cv::morphologyEx(HSV_mask, HSV_mask, 3,element);  //Close
    cv::GaussianBlur( HSV_mask, HSV_mask, cv::Size(9, 9), GaussianBlurSigma);
    ROS_INFO_STREAM("HoughCircles");
    cv::HoughCircles( HSV_mask, circles, CV_HOUGH_GRADIENT, 1, HSV_mask.rows/10, cannyThreshold, accumulatorThreshold, 0, radiusThreshold );
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
        cv::Mat roi2(depthOut_->image, r);
        ROS_INFO_STREAM("density check");
        int k = (cv::countNonZero(roi)*100.0)/(roi.cols*roi.rows);
        ROS_INFO_STREAM("Density is " << k);
        if (density < k)
        {
          ROS_INFO_STREAM("in the loooooooooop");
          density = k;
          j = i;
          target_center = center;
          target_radius = radius;
          float horizontal = center.x;
          target.x = horizontal;
          cv::Scalar tempVal = mean(roi2);
          float distance = tempVal.val[0]/1000;
          target.z = distance;

          ROS_INFO_STREAM("X " << horizontal << "Z" <<  distance);

        }

      }

       // circle center
       cv::circle( BGR, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
       // circle outline
       cv::circle( BGR, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

    }


    if(j >= 0)
    {
      ROS_INFO_STREAM("Drawing the circle");
      cv::circle( BGR_filtered, target_center, target_radius, cv::Scalar(44,55,155), 20, 12, 0 );
      ROS_INFO_STREAM("X " << target.x << "Z" <<  target.z);
      goal_pub.publish(target);

    }


    // Update GUI Window
    ROS_INFO_STREAM("updating image window");
    cv::imshow(RGB_WINDOW, BGR);
    cv::imshow(FILTER_WINDOW, BGR_filtered);
    cv::imshow(CIRCLE_WINDOW, HSV_mask);

    cv::waitKey(3);

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
