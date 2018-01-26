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

  cv::namedWindow(RGB_WINDOW);
  cv::namedWindow(DEPTH_WINDOW);
}

ImageConverter::~ImageConverter()
{
  cv::destroyWindow(RGB_WINDOW);
  cv::destroyWindow(DEPTH_WINDOW);
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
    // Update GUI Window
    cv::imshow(RGB_WINDOW, rbgOut_->image);
    cv::imshow(DEPTH_WINDOW, depthOut_->image);
    rate.sleep();
  }
}


void ImageConverter::process()
{
  // Atonomicly copy the image pointer so it doesn't change during process();
  rbgIn = rbgIn_;
  depthIn = depthIn_;

  // Check if image if rbg image has been processed before
  if(rbgOut_.header.seq == rbgIn.header.seq)
    return;


  try{
    rbgOut_ = cv_bridge::toCvCopy(rbgIn, sensor_msgs::image_encodinngs::BGR8);
    depthOut_ = cv_bridge::toCvCopy(depthIn, sensor_msgs::image_encodinngs::MONO8);
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (rbgOut_->image.rows > 60 && rbgOut_->image.cols > 60)
  {
    cv::circle(rbgIn->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
  }

  // Output modified video stream
  rbg_pub_.publish(rbgOut_->toImageMsg());
}
