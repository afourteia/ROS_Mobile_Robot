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
  cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );


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

void ImageConverter::depthCb(const sensor_msgs::ImageConstPtr& img)
{
  depthIn_ = img;
}

void ImageConverter::spin()
{
  ros::Rate rate(30); //
  // cv::createTrackbar( trackbar_type, window_name, &threshold_type,
  //   max_type, Threshold_Demo );
  //
  // cv::createTrackbar( trackbar_value,
  //   window_name, &threshold_value, max_value, Threshold_Demo );

  Threshold_Demo( 0, 0 );
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


void ImageConverter::process()
{
  // Atonomicly copy the image pointer so it doesn't change during process();
  rbgIn = rbgIn_;
  depthIn = depthIn_;

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
    cv::Mat gray_image, hsv_image, mask;
    cv::cvtColor( rbgOut_->image, gray_image, CV_BGR2GRAY );
    cv::cvtColor( rbgOut_->image, hsv_image, CV_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask);
  }

  // Output modified video stream
  image_pub_.publish(rbgOut_->toImageMsg());
}


void ImageConverter::Threshold_Demo( int, void* )
{
  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */

  threshold( src_gray, dst, threshold_value, max_BINARY_value,threshold_type );

  imshow( window_name, dst );
}
