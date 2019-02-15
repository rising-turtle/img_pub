/*
 *  July 10 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  decompress rosbag file, 
 *
 * */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

using namespace std; 
using namespace cv; 

string base_dir(""); 

void processBagfile(string bagfile); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bag_decompress_rgb_imu");
  
  ros::NodeHandle nh; 

  ROS_INFO("./bag_decompress_rgb_imu [bagfile] [output_dir]");

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

  // mkdir 
  string d_dir = base_dir;//gDataName; 
  string d_rgb = d_dir + "/color"; 
  string d_dpt = d_dir + "/depth"; 
  string d_ir = d_dir + "/ir"; 
  string d_ir2 = d_dir + "/ir2"; 
  // std::cout<<d_dir<<"   d_dir_got|||"<<d_rgb<<"   d_rgb|||"<<d_dpt<<"   d_dpt|||"<<std::endl;

  mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_rgb.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_dpt.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir2.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  
  // imu file and timestamp file 
  ofstream imu_f(d_dir + "/imu_vn100.log"); 
  ofstream img_f(d_dir + "/timestamp.txt");
  img_f<<"index\ttimestamp"<<endl;

  // for extract cv::mat 
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;
  cv_bridge::CvImageConstPtr cv_ptrIr; 
  cv_bridge::CvImageConstPtr cv_ptrIr2; 

  string last_time(""); 

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if(m.getTopic() == imu_tpc || ("/"+m.getTopic()) == imu_tpc)
    {
      // receive a imu message
      sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>(); 
      imu_f<<std::fixed<<simu->header.stamp<<"\t"<<simu->linear_acceleration.x << "\t"<<simu->linear_acceleration.y << "\t"
        <<simu->linear_acceleration.z<<"\t"<<simu->angular_velocity.x<<"\t"<<simu->angular_velocity.y<<"\t"
        <<simu->angular_velocity.z<<"\t"<<0.0<<"\t"<<0.0<<"\t"<<0.0<<endl;
      
      cout<<"bag_decompress.cpp: receive an IMU msg at time: "<<simu->header.stamp<<endl;

    }else // an image mssage 
    {
      sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
      stringstream tt; 
      tt << simage->header.stamp;
      // cout << "bag_decompress.cpp: receive an image msg at time: "<<tt.str()<<endl; 
      cout << "bag_decompress.cpp: receive an image msg at time: "<<simage->header.stamp<<endl;

      if(last_time == "" || tt.str() != last_time) // first timestamp 
      {
        last_time = tt.str(); 
        // img_f<<last_time<<"\tcolor/"<<last_time<<".png"<<"\t"<<last_time<<"\tdepth/"<<last_time<<".exr"<<"\t"
        //  <<last_time<<"\tir/"<<last_time<<".png"<<"\t"<<last_time<<"\tir2/"<<last_time<<".png"<<endl;
        img_f<<last_time<<"\tcolor/"<<last_time<<".png"<<"\t"<<last_time<<"\tdepth/"<<last_time<<".png"<<"\t"
          <<last_time<<"\tir/"<<last_time<<".png"<<"\t"<<last_time<<"\tir2/"<<last_time<<".png"<<endl;
      }      
      
      if(m.getTopic() == rgb_tpc || ("/"+m.getTopic()) == rgb_tpc)
      {
        // receive a rgb image 
        cv_ptrRGB = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::BGR8); 
        imwrite(d_rgb + "/"+ tt.str()+".png", cv_ptrRGB->image); 
	imshow("rgb_file", cv_ptrRGB->image); 
	waitKey(3);
      }
      if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc)
      {
        // receive a dpt image
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1); 
        imwrite(d_dpt + "/"+tt.str() +".png", cv_ptrD->image); 
        // imshow("dpt_file", cv_ptrD->image); 
        waitKey(3); 
      }
      if(m.getTopic() == ir_tpc || ("/"+m.getTopic()) == ir_tpc)
      {
        // receive a ir image 
        // cv_ptrIr = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_8UC1); 
        // imwrite(d_ir + "/"+tt.str() +".png", cv_ptrIr->image); 
      }
      if(m.getTopic() == ir2_tpc || ("/"+m.getTopic()) == ir2_tpc)
      {
        // receive a ir2 image
        // cv_ptrIr2 = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_8UC1); 
        // imwrite(d_ir2 + "/" + tt.str()+".png", cv_ptrIr2->image); 
      }
    }
  }
  return ; 
}


