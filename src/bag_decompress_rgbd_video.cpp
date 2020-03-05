/*
 *  Feb. 4 2020, He Zhang, hxzhang1@ualr.edu
 *
 *  decompress rosbag file, and save the color and depth image into video 
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

#include "opencv2/opencv.hpp"
#include "opencv/cv.h"

using namespace std; 
using namespace cv; 

string base_dir(""); 

void processBagfile(string bagfile); 
cv::Mat covert_to_color(const cv::Mat& d);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bag_decompress_rgbd_video");
  
  ros::NodeHandle nh; 

  ROS_INFO("./bag_decompress_rgbd_video [bagfile] [output_dir]");

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
  // topics.push_back(ir_tpc);
  // topics.push_back(ir2_tpc);
  // topics.push_back(imu_tpc); 

  rosbag::Bag bag; 
  bag.open(bagfile, rosbag::bagmode::Read); 
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // mkdir 
  std::size_t found = bagfile.find_last_of("/"); 
  std::size_t found_dash = bagfile.find_last_of("."); 
  std::size_t last = found_dash - found - 1; // bagfile.size()-4; 
  string name = bagfile.substr(found+1, last); 
  string d_dir = base_dir;//gDataName; 
  string d_rgb = d_dir + "/" + name+"_color.avi"; 
  string d_dpt = d_dir + "/" + name+"_depth.avi"; 
  // string d_ir = d_dir + "/ir"; 
  // string d_ir2 = d_dir + "/ir2"; 
  // std::cout<<d_dir<<"   d_dir_got|||"<<d_rgb<<"   d_rgb|||"<<d_dpt<<"   d_dpt|||"<<std::endl;

  mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_rgb.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_dpt.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir2.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  
  // imu file and timestamp file 
  // ofstream imu_f(d_dir + "/imu_vn100.log"); 
  // ofstream img_f(d_dir + "/timestamp.txt");
  // img_f<<"index\ttimestamp"<<endl;

  // for extract cv::mat 
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;
  cv_bridge::CvImageConstPtr cv_ptrIr; 
  cv_bridge::CvImageConstPtr cv_ptrIr2; 

  string last_time(""); 

  // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
  // Define the fps to be equal to 10. Also frame size is passed.

  VideoWriter rgb_video(d_rgb.c_str(),CV_FOURCC('M','J','P','G'), 30, Size(640,480));
  VideoWriter dpt_video(d_dpt.c_str(),CV_FOURCC('M','J','P','G'), 30, Size(640,480));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if(m.getTopic() == imu_tpc || ("/"+m.getTopic()) == imu_tpc)
    {
      // receive a imu message
      sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>(); 
      // imu_f<<std::fixed<<simu->header.stamp<<"\t"<<simu->linear_acceleration.x << "\t"<<simu->linear_acceleration.y << "\t"
      //  <<simu->linear_acceleration.z<<"\t"<<simu->angular_velocity.x<<"\t"<<simu->angular_velocity.y<<"\t"
      // <<simu->angular_velocity.z<<"\t"<<0.0<<"\t"<<0.0<<"\t"<<0.0<<endl;
      
      cout<<"bag_decompress_rgbd_video.cpp: receive an IMU msg at time: "<<simu->header.stamp<<endl;

    }else // an image mssage 
    {
      sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
      stringstream tt; 
      tt << simage->header.stamp;
      // cout << "bag_decompress.cpp: receive an image msg at time: "<<tt.str()<<endl; 
      cout << "bag_decompress_rgbd_video.cpp: receive an image msg at time: "<<simage->header.stamp<<endl;
      
      if(m.getTopic() == rgb_tpc || ("/"+m.getTopic()) == rgb_tpc)
      {
        // receive a rgb image 
        // cv_ptrRGB = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::BGR8); 
        cv_ptrRGB = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_8UC3); 

        // imwrite(d_rgb + "/"+ tt.str()+".png", cv_ptrRGB->image); 
        // imshow("rgb_file", cv_ptrRGB->image); 
        rgb_video.write(cv_ptrRGB->image);
        waitKey(3);
      }
      if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc)
      {

        // receive a dpt image
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1); 
        // imwrite(d_dpt + "/"+tt.str() +".png", cv_ptrD->image); 
        cv::Mat psudo = covert_to_color(cv_ptrD->image); 
        imshow("dpt_file", psudo); //cv_ptrD->image); 
        dpt_video.write(psudo); 
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
  rgb_video.release();
  dpt_video.release();
  return ; 
}



cv::Mat covert_to_color(const cv::Mat& d)
{
  cv::Mat color= cv::Mat(d.rows, d.cols, CV_8UC1, Scalar(0)); 

  double MAX_STD = 7.; 

  for(int r=0; r<d.rows; r++)
  for(int c=0; c<d.cols; c++){
    double std = d.at<unsigned short>(r,c)*0.001; 
    if(std <= 0.4) std = 0; 
    if(std >= MAX_STD) std = MAX_STD; 
    double ratio = std / MAX_STD;
    ratio = ratio>1? 1.:ratio;
    color.at<unsigned char>(r,c) = (unsigned char)( ratio * 255);
  }

  Mat cm_img0;
    // applyColorMap(color, cm_img0, COLORMAP_HOT);
    applyColorMap(color, cm_img0, COLORMAP_JET);
    // imwrite(out_img, G); 

  return cm_img0;
}