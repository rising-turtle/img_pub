/*
 *  Jun. 6 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish img in ros 
 *
 * */

#include "img_pub.h"
#include <cv_bridge/cv_bridge.h>

CImgPub::CImgPub(): 
  mb_data_ready(false),
  m_curr_idx(0)
{
  ros::NodeHandle n; 
  int q = 7; 
  m_rgb_pub = n.advertise<sensor_msgs::Image>("/rgb", q); 
  m_dpt_pub = n.advertise<sensor_msgs::Image>("/dpt", q); 
  m_ir1_pub = n.advertise<sensor_msgs::Image>("/ir1", q); 
  m_ir2_pub = n.advertise<sensor_msgs::Image>("/ir2", q); 

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

bool CImgPub::getNextIr1(cv::Mat& ir)
{
  ROS_WARN("img_pub.cpp: in CImgPub getNextIr1(), should be reimplemented!"); 
  return false; 
}

bool CImgPub::getNextIr2(cv::Mat& ir)
{
  ROS_WARN("img_pub.cpp: in CImgPub getNextIr2(), should be reimplemented!"); 
  return false; 
}


bool CImgPub::publishRGBD_IR2()
{
  if(mb_data_ready == false)
  {
    ROS_ERROR("img_pub.cpp: mb_data_ready = false, need to load data first!"); 
    return false; 
  }
  
  cv::Mat rgb, dpt, ir1, ir2; 
  bool getNewRgb = getNextRGB(rgb); 
  bool getNewDpt = getNextDpt(dpt); 
  bool getNewIr1 = getNextIr1(ir1); 
  bool getNewIr2 = getNextIr2(ir2); 

  if(getNewRgb == false || getNewDpt == false || getNewIr1 == false || getNewIr2 == false)
  {
    ROS_WARN("img_pub.cpp: get next data failed, maybe finished! replay?"); 
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
  
  // ir msg
  cv_bridge::CvImage ir1_msg; 
  ir1_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; 
  ir1_msg.header.stamp = timestamp; 
  ir1_msg.image = ir1; 

  // ir msg
  cv_bridge::CvImage ir2_msg; 
  ir2_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; 
  ir2_msg.header.stamp = timestamp; 
  ir2_msg.image = ir2; 

  // publish data 
  m_rgb_pub.publish(rgb_msg); 
  m_dpt_pub.publish(dpt_msg); 
  m_ir1_pub.publish(ir1_msg); 
  m_ir2_pub.publish(ir2_msg); 

  // ros RR 
  ros::spinOnce(); 
  mp_pub_rate->sleep(); 
  moveNext(); 

  return true; 
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
  moveNext(); 

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
  moveNext(); 

  return true; 
} 
bool CImgPub::moveNext()  // move current idx to the next 
{
  ++m_curr_idx; 
  return true;
}


