/*
 *  Aug. 9 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  decompress rosbag file, only extract IMU msgs
 *
 * */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

using namespace std; 

string base_dir(""); 

void processBagfile(string bagfile); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bag_decompress_imu");
  
  ros::NodeHandle nh; 

  ROS_INFO("./bag_decompress_imu [bagfile] [file]");

  string bagfile = "";
  if(argc >= 2) 
    bagfile = argv[1]; 
  if(argc >= 3)
    base_dir = argv[2]; 

  processBagfile(bagfile); 

  return 0; 
}

void processBagfile(string bagfile)
{
  std::vector<std::string> topics;
  string rgb_tpc = "/cam0/color"; // "/cam0/image_raw";   
  string dpt_tpc = "/cam0/depth"; 
  string ir_tpc  = "/cam0/ir";
  string ir2_tpc = "/cam0/ir2"; 
  string imu_tpc = "/imu0";
  topics.push_back(rgb_tpc); 
  topics.push_back(dpt_tpc); 
  topics.push_back(ir_tpc);
  topics.push_back(ir2_tpc);
  topics.push_back(imu_tpc); 

  rosbag::Bag bag; 
  bag.open(bagfile, rosbag::bagmode::Read); 
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  ROS_INFO("bag_decompress_imu.cpp: load bag, start to work!");

  // mkdir 
  string d_dir = base_dir;//gDataName; 
  // std::cout<<d_dir<<"   d_dir_got|||"<<d_rgb<<"   d_rgb|||"<<d_dpt<<"   d_dpt|||"<<std::endl;

  // mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
 
  // imu file and timestamp file 
  ofstream imu_f(d_dir); //(d_dir + "/imu_vn100.log"); 
  if(!imu_f.is_open())
  {
    ROS_ERROR("bag_compress_imu.cpp: failed to open file %s", d_dir.c_str());
    return ; 
  }
  string last_time(""); 

  ros::Time pre_t; 
  ros::Time cur_t; 
  bool first = true; 
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if(m.getTopic() == imu_tpc || ("/"+m.getTopic()) == imu_tpc)
    {
      // receive a imu message
      sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>(); 
      imu_f<<std::fixed<<simu->header.stamp<<"\t"<<simu->linear_acceleration.x << "\t"<<simu->linear_acceleration.y << "\t"
        <<simu->linear_acceleration.z<<"\t"<<simu->angular_velocity.x<<"\t"<<simu->angular_velocity.y<<"\t"
        <<simu->angular_velocity.z<<"\t"<<0.0<<"\t"<<0.0<<"\t"<<0.0<<endl;
      if(first)
      {
        pre_t = cur_t = simu->header.stamp; 
        first = false; 
      }else
      {
        cur_t = simu->header.stamp; 
        ros::Duration dt = cur_t - pre_t; 
        pre_t = cur_t; 
        double dt_ms = dt.toNSec()*1e-6; 
        if(dt_ms < 5.1)
        {
          // cout<<"bag_decompress.cpp: receive an IMU msg at time: "<<simu->header.stamp<<endl;
          ROS_INFO_STREAM_THROTTLE(10, "bag_decompress_imu.cpp: IMU msg at time "<<cur_t<<" dt_ms = "<<dt_ms); 
        }else
        {
          ROS_WARN_STREAM("bag_decompress_imu.cpp: IMU msg at time "<<cur_t<<" dt_ms = "<<dt_ms); 
          // sleep(10); 
        }
      }

    }else // an image mssage 
    {
    }
  }
  return ; 
}


