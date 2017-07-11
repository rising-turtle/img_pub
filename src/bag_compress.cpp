/*
 *  July 11 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  compress realsense dataset into a bag file 
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

bool readImuData(string fname, vector<string>& timestamp, vector<vector<double> >& data); 

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

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bag_compress"); 
  ros::NodeHandle nh;
  ros::NodeHandle np("~"); 
  if(argc >= 2)
    data_dir = argv[1]; 
  if(argc >= 3)
    bag_file = argv[2]; 
  
  // topics 
  string rgb_tpc = "/cam0/color"; 
  string dpt_tpc = ""; //"/cam0/depth";
  string ir_tpc = ""; // "/cam0/ir";
  string ir2_tpc = ""; // "/cam0/ir2"; 
  string imu_tpc = "/imu0"; 

  // get image directories 
  CImgPubRS pub; 
  pub.getData(data_dir); 

  // rosbag for write 
  rosbag::Bag bag; 
  bag.open(bag_file, rosbag::bagmode::Write); 

  // process IMU msgs 
  vector<vector<double> > m; 
  vector<string> mt; 
  if(!readImuData(data_dir + "/imu_vn100.log", mt, m))
  {
    return -1;
  }
  
  // write image information 
  cv::Mat rgb, dpt, ir, ir2;
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

    // write imu msgs 
    while(i < m.size())
    {
      ros::Time imuT = compT(mt[i]);
      if(rost < imuT) 
        break; // image earlier than imu, write image 
      // imu earlier than image, write imu 
      
      // imu msg
      sensor_msgs::Imu imu_msg; 
      imu_msg.header.stamp = imuT; 
      imu_msg.linear_acceleration.x = m[i][0]; 
      imu_msg.linear_acceleration.y = m[i][1];
      imu_msg.linear_acceleration.z = m[i][2]; 

      imu_msg.angular_velocity.x = m[i][3]; 
      imu_msg.angular_velocity.y = m[i][4]; 
      imu_msg.angular_velocity.z = m[i][5]; 

      bag.write(imu_tpc, imuT, imu_msg); 
      i++; // next imu msg 
    }

    cv::imshow("rgb image", rgb); 
    cv::waitKey(1); 

    //rgb msg
    if(rgb_tpc != "")
    {
      cv_bridge::CvImage rgb_msg; 
      rgb_msg.encoding = sensor_msgs::image_encodings::BGR8; 
      rgb_msg.header.stamp = rost;
      rgb_msg.image = rgb; 
      bag.write(rgb_tpc, rost, rgb_msg); 
    }
    if(dpt_tpc != "" && pub.getNextDpt(dpt))
    {
      cv_bridge::CvImage dpt_msg; 
      dpt_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; 
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

  ROS_INFO("bag_compress.cpp: finish writing image msgs!"); 

  // write the rest imu msgs 
  while(i < m.size())
  {
    ros::Time imuT = compT(mt[i]);
    
    // imu msg
    sensor_msgs::Imu imu_msg; 
    imu_msg.header.stamp = imuT; 
    imu_msg.linear_acceleration.x = m[i][0]; 
    imu_msg.linear_acceleration.y = m[i][1];
    imu_msg.linear_acceleration.z = m[i][2]; 

    imu_msg.angular_velocity.x = m[i][3]; 
    imu_msg.angular_velocity.y = m[i][4]; 
    imu_msg.angular_velocity.z = m[i][5]; 

    bag.write(imu_tpc, imuT, imu_msg); 
    i++; // next imu msg 
  }

  ROS_INFO("bag_compress.cpp: finish writing IMU msgs!"); 

  bag.close(); 
  return 0; 
}


bool readImuData(string fname, vector<string>& timestamp, vector<vector<double> >& data)
{
  ifstream inf(fname.c_str()); 

  if(!inf.is_open())
  {
    ROS_ERROR("bag_compress.cpp: failed to open file %s", fname.c_str()); 
    return false; 
  }

  string t; 
  float ax, ay, az, gx, gy, gz, yaw, pitch, roll; 
  vector<double> m(6,0.); 
  string s; 
  while(!inf.eof())
  {
    getline(inf, s); 
    if(s.empty()) break; 
    stringstream ss; 
    ss << s;
    ss>>t>>ax>>ay>>az>>gx>>gy>>gz>>yaw>>pitch>>roll; 
    m[0] = ax; m[1] = ay; m[2] = az; 
    m[3] = gx; m[4] = gy; m[5] = gz; 
    timestamp.push_back(t); 
    data.push_back(m); 
  }
  ROS_INFO("%s succeed to load %i imu measurements", __FILE__, data.size()); 
  inf.close(); 
  return true; 
}

