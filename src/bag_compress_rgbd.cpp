/*
 *  Feb. 4 2019, He Zhang, hzhang8@vcu.edu
 *
 *  compress rgbd dataset into a bag file 
 *
 * */

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp> 
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include "img_pub_realsense.h"

using namespace std; 

string data_dir(""); 
string bag_file(""); 

bool operator<(const ros::Time& t1, const ros::Time& t2)
{
  return ((t1.sec<t2.sec) || (t1.sec==t2.sec && t1.nsec < t2.nsec));
}

ros::Time compT(string curr_t)
{
    string secs = curr_t.substr(0, 10); 
    string nanosecs = curr_t.substr(11, 9); 
    ros::Time rost(std::stoi(secs), std::stoi(nanosecs)); 
    return rost; 
}

ros::Time compT6(string curr_t)
{
    string secs = curr_t.substr(0, 10); 
    string nanosecs = curr_t.substr(11, 6); 
    ros::Time rost(std::stoi(secs), std::stoi(nanosecs)*1000.); 
    return rost; 
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bag_compress_rgbd"); 
  ros::NodeHandle nh;
  ros::NodeHandle np("~"); 
  if(argc <=2)
  {
    ROS_WARN("usage: ./bag_compress [data_dir] [bag_name]");
    return 0; 
  }
  if(argc >= 2)
    data_dir = argv[1]; 
  if(argc >= 3)
    bag_file = argv[2]; 
  
  // topics 
  string rgb_tpc = "/cam0/color"; 
  string dpt_tpc = "/cam0/depth";
  string ir_tpc =  ""; // "/cam0/ir";
  string ir2_tpc = ""; // "/cam0/ir2"; 

  // get image directories 
  CImgPubRS pub; 
  pub.getData(data_dir); 

  // rosbag for write 
  rosbag::Bag bag; 
  bag.open(bag_file, rosbag::bagmode::Write); 

  // write image information 
  cv::Mat rgb, dpt, ir, ir2;
  int rgb_type; 
  int i=0; 
  do
  {
    bool getNewRgb = pub.getNextRGB(rgb);  
    if(getNewRgb== false) break; 
    
    // timestamp 
    std::string curr_t; 
    pub.getNextTimeStamp(curr_t); 
    // string secs = curr_t.substr(0, 10); 
    // string nanosecs = curr_t.substr(11, 9); 
    // ros::Time rost(std::stoi(secs), std::stoi(nanosecs)); 
    ros::Time rost = compT(curr_t); 

    cv::imshow("rgb image", rgb); 
    cv::waitKey(1); 

    //rgb msg
    if(rgb_tpc != "")
    {
      cv_bridge::CvImage rgb_msg; 
      rgb_type = rgb.type(); 
      if(rgb_type == CV_8UC1) // only check 8UC1, future 16UC1
	rgb_msg.encoding = sensor_msgs::image_encodings::MONO8; 
      else
	rgb_msg.encoding = sensor_msgs::image_encodings::BGR8; 
      rgb_msg.header.stamp = rost;
      rgb_msg.image = rgb; 
      cout <<"write rgb timestamp = "<<std::fixed<<rost.toSec()<<" string = "<<curr_t<<endl;
      bag.write(rgb_tpc, rost, rgb_msg); 
    }
    if(dpt_tpc != "" && pub.getNextDpt(dpt))
    {
      cv_bridge::CvImage dpt_msg; 
      // dpt_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; 
      dpt_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; 
      dpt_msg.header.stamp = rost; 
      dpt_msg.image = dpt; 
      bag.write(dpt_tpc, rost, dpt_msg); 
    }
    if(ir_tpc != "" && pub.getNextIr1(ir))
    {
      cv_bridge::CvImage ir1_msg; 
      ir1_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; 
      ir1_msg.header.stamp = rost; 
      ir1_msg.image = ir; 
      bag.write(ir_tpc, rost, ir1_msg); 
    }
    if(ir2_tpc != "" && pub.getNextIr2(ir2))
    {
      cv_bridge::CvImage ir2_msg; 
      ir2_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; 
      ir2_msg.header.stamp = rost; 
      ir2_msg.image = ir2; 
      bag.write(ir2_tpc, rost, ir2_msg); 
    }
    
    // move to next 
    pub.moveNext(); 
    usleep(10); 

  }while(ros::ok()); 

  ROS_INFO("bag_compress_rgbd.cpp: finish writing image msgs!"); 

  bag.close(); 
  return 0; 
}

