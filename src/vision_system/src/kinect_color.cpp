#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>


static const std::string RGB_WINDOW = "rgb window";
static const std::string FILTER_WINDOW = "filtered window";
/// Global Variables
int HueH = 255;
int HueL = 0;
int SatH = 255;
int SatL = 0;
int ValH = 255;
int ValL = 0;


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

  cv::Mat HSV;
  cv::Mat BGR_filtered;




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

    cv::namedWindow(RGB_WINDOW, 1);
    cv::namedWindow(FILTER_WINDOW, 1);

    cv::createTrackbar("Hue Max threshold", FILTER_WINDOW, &HueH, HueH);
    cv::createTrackbar("Hue Max threshold", FILTER_WINDOW, &HueL, HueH);
    cv::createTrackbar("Sat Max threshold", FILTER_WINDOW, &SatH, SatH);
    cv::createTrackbar("Sat Max threshold", FILTER_WINDOW, &SatL, SatH);
    cv::createTrackbar("Value Max threshold", FILTER_WINDOW, &ValH, ValH);
    cv::createTrackbar("Value Max threshold", FILTER_WINDOW, &ValL, ValH);

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
    cv::cvtColor(rbgOut_->image, HSV, CV_BGR2HSV);
    cv::inRange(HSV, cv::Scalar(HueL,SatL,ValL),cv::Scalar(HueH,SatH,ValH),HSV);
    cv::cvtColor(HSV, BGR_filtered, CV_HSV2BGR);

    // Update GUI Window
    ROS_INFO_STREAM("updating image window");
    cv::imshow(RGB_WINDOW, rbgOut_->image);
    cv::imshow(FILTER_WINDOW, BGR_filtered);

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
