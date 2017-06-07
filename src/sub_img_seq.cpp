/*
 *  June 7 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  subscribe img, and show it 
 *
 * */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void rgbCb(const sensor_msgs::ImageConstPtr& msgRGB)
{
  ROS_WARN_ONCE_NAMED("sub_img_seq", "First rgb msg received by rgbCb "); 

  cv_bridge::CvImageConstPtr cv_ptrRGB; 
  cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8); 
  
  cv::imshow( " rgb ", cv_ptrRGB->image);
  cv::waitKey(3); 
  return ; 
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sub_img_seq"); 
  ros::NodeHandle n; 
  
  ros::Subscriber img_sub_ = n.subscribe<sensor_msgs::Image>("/rgb", 1, rgbCb);

  ros::spin(); 
  return 1; 
}

