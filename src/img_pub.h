/*
 *  Jun. 6 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish img in ros 
 *
 * */

#pragma once 

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>

class CImgPub 
{
public:
  CImgPub(); 
  virtual ~CImgPub(); 

  virtual bool getData(std::string dir); 
  virtual bool getNextRGB(cv::Mat& );
  virtual bool getNextDpt(cv::Mat& ); 
  
  // publish keyframe data 
  ros::Publisher m_rgb_pub; // publish rgb 
  ros::Publisher m_dpt_pub; // publish dpt 
  ros::Rate* mp_pub_rate;   // publish rate 

  bool publishRGBD(); 
  bool publishRGB(); 
  bool mb_data_ready; 
  std::string m_data_dir;   // where data is stored 
  int m_curr_idx;           // point to the index of current frame
};


