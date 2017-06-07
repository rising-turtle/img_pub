/*
 *  Jun. 6 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish img in ros 
 *
 * */

#include "img_pub.h"
#include <cv_bridge/cv_bridge.h>

CImgPub::CImgPub(): 
  mb_data_ready(false)
{
  ros::NodeHandle n; 
  int q = 7; 
  m_rgb_pub = n.advertise<sensor_msgs::Image>("/rgb", q); 
  m_dpt_pub = n.advertise<sensor_msgs::Image>("/dpt", q); 
  
  ros::NodeHandle np("~"); 
  int publish_rate = 30; 
  np.param("publish_rate", publish_rate, publish_rate); 
  mp_pub_rate = new ros::Rate(publish_rate); 
}
CImgPub::~CImgPub()
{
  if(mp_pub_rate) 
  {
    delete mp_pub_rate; 
    mp_pub_rate = NULL; 
  }
}

bool CImgPub::getData(std::string dir)
{
  ROS_WARN("img_pub.cpp: In CImgPub getData(), should be reimplemented!"); 
  return false; 
}

bool CImgPub::getNextRGB(cv::Mat& rgb)
{
  ROS_WARN("img_pub.cpp: in CImgPub getNextRGB(), should be reimplemented!"); 
  return false; 
}

bool CImgPub::getNextDpt(cv::Mat& dpt)
{
  ROS_WARN("img_pub.cpp: in CImgPub getNextDpt(), should be reimplemented!"); 
  return false; 
}

bool CImgPub::publishRGBD()
{
  if(mb_data_ready == false)
  {
    ROS_ERROR("img_pub.cpp: mb_data_ready = false, need to load data first!"); 
    return false; 
  }
  
  cv::Mat rgb, dpt; 
  bool getNewRgb = getNextRGB(rgb); 
  bool getNewDpt = getNextDpt(dpt); 
  
  if(getNewRgb == false || getNewDpt == false)
  {
    ROS_WARN("img_pub.cpp: all data has published, need to replay?"); 
    return false; 
  }

  ros::Time timestamp = ros::Time::now(); 

  // rgb msg 
  cv_bridge::CvImage rgb_msg; 
  rgb_msg.encoding = sensor_msgs::image_encodings::BGR8; 
  rgb_msg.header.stamp = timestamp;
  rgb_msg.image = rgb; 
  
  // dpt msg 
  cv_bridge::CvImage dpt_msg; 
  dpt_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; 
  dpt_msg.header.stamp = timestamp; 
  dpt_msg.image = dpt; 
  
  // publish data 
  m_rgb_pub.publish(rgb_msg); 
  m_dpt_pub.publish(dpt_msg); 

  // ros RR 
  ros::spinOnce(); 
  mp_pub_rate->sleep(); 

  return true; 
}

bool CImgPub::publishRGB()
{
  if(mb_data_ready == false)
  {
    ROS_ERROR("img_pub.cpp: mb_data_ready = false, need to load data first!"); 
    return false; 
  }
  
  cv::Mat rgb; 
  bool getNewRgb = getNextRGB(rgb); 
 
  if(getNewRgb == false)
  {
    ROS_WARN("img_pub.cpp: all data has published, need to replay?"); 
    return false; 
  }

  ros::Time timestamp = ros::Time::now(); 

  // rgb msg 
  cv_bridge::CvImage rgb_msg; 
  rgb_msg.encoding = sensor_msgs::image_encodings::BGR8; 
  rgb_msg.header.stamp = timestamp;
  rgb_msg.image = rgb; 
  
  // publish data 
  m_rgb_pub.publish(rgb_msg); 
  
  // ros RR 
  ros::spinOnce(); 
  mp_pub_rate->sleep(); 

  return true; 
} 

