#include "image_processing.h"


ImageConverter::ImageConverter()
 : it_(nh_)
{
  // Subscribe to input video feed and publish output video feed
  rbg_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,
    &ImageConverter::rbgCb, this);
  depth_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1,
    &ImageConverter::depthCb, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);

  cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter()
{
  cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::rbgCb(const sensor_msgs::ImageConstPtr& img)
{
  rbgIn_ = img;
}

void ImageConverter::rbgCb(const sensor_msgs::ImageConstPtr& img)
{
  depthIn_ = img;
}

void ImageConverter::spin()
{
  ros::Rate rate(30);
  while(ros::ok())
  {
    process();
    spinOnce();
    rate.sleep();
  }
}


void ImageConverter::process()
{
  cv_bridge::CvImagePtr cv_ptr;

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodinngs::BGR8);
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  {
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
  }

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}
