/*
 *  June 7 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  subscribe img: rgb, dpt, ir1, ir2, and show it 
 *
 * */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, 
        sensor_msgs::Image, sensor_msgs::Image> rgbdIr2SyncPolicy;


void rgbdIRCb(const sensor_msgs::ImageConstPtr& msgRGB,  const sensor_msgs::ImageConstPtr& msgD,
   const sensor_msgs::ImageConstPtr& msgIr1, const sensor_msgs::ImageConstPtr& msgIr2)
{
  ROS_WARN_ONCE_NAMED("sub_img_realsense", "First rgbdir msg received by rgbdIrCb "); 

  cv_bridge::CvImageConstPtr cv_ptrRGB, cv_ptrD, cv_ptrIr1, cv_ptrIr2; 
  cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8); 
  cv_ptrD = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_32FC1); 
  cv_ptrIr1 = cv_bridge::toCvShare(msgIr1, sensor_msgs::image_encodings::TYPE_8UC1); 
  cv_ptrIr2 = cv_bridge::toCvShare(msgIr2, sensor_msgs::image_encodings::TYPE_8UC1); 

  cv::imshow( " rgb ", cv_ptrRGB->image);
  cv::waitKey(3); 
  cv::imshow(" dpt ", cv_ptrD->image);
  cv::waitKey(3); 
  cv::imshow(" ir1 ", cv_ptrIr1->image); 
  cv::waitKey(3); 
  cv::imshow(" ir2 ", cv_ptrIr2->image); 
  cv::waitKey(3); 
  return ; 
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sub_img_seq"); 
  ros::NodeHandle n; 
  
  int q = 7; 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, "/rgb", q); 
  message_filters::Subscriber<sensor_msgs::Image> dpt_sub(n, "/dpt", q); 
  message_filters::Subscriber<sensor_msgs::Image> ir1_sub(n, "/ir1", q); 
  message_filters::Subscriber<sensor_msgs::Image> ir2_sub(n, "/ir2", q); 

  message_filters::Synchronizer<rgbdIr2SyncPolicy> sync(rgbdIr2SyncPolicy(q), rgb_sub, dpt_sub, ir1_sub, ir2_sub); 
  sync.registerCallback(boost::bind(&rgbdIRCb, _1, _2, _3, _4));

  ros::spin(); 
  return 1; 
}

