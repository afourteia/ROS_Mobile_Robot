#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static const std::string RGB_WINDOW = "rgb window";
static const std::string DEPTH_WINDOW = "depth window";
static const std::string window_name = "Threshold Demo";


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

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    rbg_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::rbgCb, this);

    depth_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1,
      &ImageConverter::depthCb, this);

    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(RGB_WINDOW);
    cv::namedWindow(DEPTH_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
    cv::destroyWindow(DEPTH_WINDOW);
  }

  void rbgCb(const sensor_msgs::ImageConstPtr& img)
  {
    rbgIn_ = img;
  }


  void depthCb(const sensor_msgs::ImageConstPtr& img)
  {
    depthIn_ = img;
  }


  void process()
  {
    // Atonomicly copy the image pointer so it doesn't change during process();
    sensor_msgs::ImageConstPtr rbgIn = rbgIn_;
    sensor_msgs::ImageConstPtr depthIn = depthIn_;

    // Check if image if rbg image has been processed before
    if(rbgOut_->header.seq == rbgIn->header.seq)
      return;


    try{
      rbgOut_ = cv_bridge::toCvCopy(rbgIn, sensor_msgs::image_encodings::BGR8);
      depthOut_ = cv_bridge::toCvCopy(depthIn, sensor_msgs::image_encodings::MONO8);
    }catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (rbgOut_->image.rows > 60 && rbgOut_->image.cols > 60)
    {
      // cv::Mat gray_image, hsv_image, mask;
      // cv::cvtColor( rbgOut_->image, gray_image, CV_BGR2GRAY );
      // cv::cvtColor( rbgOut_->image, hsv_image, CV_BGR2HSV);
      // cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask);
      cv::circle(rbgOut_->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    }

    // Output modified video stream
    image_pub_.publish(rbgOut_->toImageMsg());
  }

  void spin()
  {
    ros::Rate rate(30);
    while(ros::ok())
    {
      process();
      ros::spinOnce();
      // Update GUI Window
      cv::imshow(RGB_WINDOW, rbgOut_->image);
      cv::imshow(DEPTH_WINDOW, depthOut_->image);
      rate.sleep();
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ic.spin();
  return 0;
}
