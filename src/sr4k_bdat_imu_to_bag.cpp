/*
 *  April 6 10 2020, He Zhang, hzhang8@vcu.edu
 *
 *  decompress sr4k's data + imu into rosbag 
 *
 * */

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <iomanip>
#include <sys/stat.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
// #include <filesystem> 
#include "sr4k_reader.h"

using namespace std; 

string sr4k_folder(""); 
string bag_file("");
string g_img_time_file("");
string g_imu_file(""); 

ros::Time compT6(string curr_t)
{
    string secs = curr_t.substr(0, 10); 
    string nanosecs = curr_t.substr(11, 6); 
    ros::Time rost(std::stoi(secs), std::stoi(nanosecs)*1000.); 
    return rost; 
}

int find_index(vector<double>& imu_times, int pre_index, double cur_img_tp); 
bool read_timestamp(string ts_file, map<string, string> & files);
bool loadImgTime(map<int, double>& mt); // load synchronized time stamps
bool loadIMUFile(vector<double>& timestamp, vector<vector<double> >& vimus);

void do_it(); 

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "sr4k_bdat_imu_to_bag"); 
  	ros::NodeHandle nh;
  	ros::NodeHandle np("~"); 
	if(argc < 3){
		cout<<" usage: ./sr4k_bdat_imu_to_bag [sr4k_folder] [bagfile] "<<endl; 
		return -1; 
	}

	// directories and timestamp files 
	sr4k_folder = argv[1]; 
	bag_file = argv[2]; 
	g_img_time_file = sr4k_folder + "/sr4k/timestamp.log"; 
	g_imu_file = sr4k_folder + "/imu_v100.log"; 

	do_it(); 

	return 0; 
}

void do_it()
{
	// rosbag for write 
  	rosbag::Bag bag; 
  	bag.open(bag_file, rosbag::bagmode::Write); 

	// topics 
  	string rgb_tpc = "/cam0/color"; 
  	string dpt_tpc = "/cam0/depth";
  	string imu_tpc = "/imu"; 

	// 1. load rgb and depth images from sr4k data 
  	// sequential id and timestamp 
	// map<int, double> img_times; 
	map<string, string> img_times; 

	if(!read_timestamp(g_img_time_file, img_times)){
	// if(!loadImgTime(img_times)){
		ROS_ERROR("sr4k_bdat_imu_to_bag.cpp: failed to read time file %s", g_img_time_file.c_str()); 
		return ; 
	}

	// 2. load imu's data 
	vector<double> imu_times; 
	vector<vector<double>> vimus; 
	if(!loadIMUFile(imu_times, vimus)){
		ROS_ERROR("sr4k_bdat_imu_to_bag.cpp: failed to read imu file: %s", g_imu_file.c_str());
		return; 
	}

	// 3. write the measurement into bag file 
	// try to convert 
	SReader sr4k; 
	map<string, string>::iterator it = img_times.begin(); 

	int pre_index = -1 ; 
	int cur_index = 0 ; 

	while(it!= img_times.end()){

		string bdat_file = sr4k_folder + "/sr4k/d1_"+it->first+".bdat"; 

		// read file 
		cv::Mat gray_img, dpt_img; 
		if(!sr4k.readBdatFrame(bdat_file, gray_img, dpt_img)) break; 

		string time_str = it->second; // timestamp 
		ros::Time rost = compT6(time_str); 

		// write imu measurement 
		{
			double cur_img_tp = rost.toSec(); 
			cur_index  = find_index(imu_times, pre_index, cur_img_tp); 
			if(cur_index > pre_index){
				for(int i=pre_index+1; i<= cur_index; i++){
					sensor_msgs::Imu imu_msg; 
					imu_msg.header.stamp.fromSec(imu_times[i]); 
					imu_msg.linear_acceleration.x = vimus[i][3];
					imu_msg.linear_acceleration.y = vimus[i][4];
					imu_msg.linear_acceleration.z = vimus[i][5];
					imu_msg.angular_velocity.x = vimus[i][0]; 
					imu_msg.angular_velocity.y = vimus[i][1]; 
					imu_msg.angular_velocity.z = vimus[i][2];
					ros::Time rost_imu = imu_msg.header.stamp; //  rost_imu.fromSec(imu_times[i]); 
					bag.write(imu_tpc, rost_imu, imu_msg); 
				}
				pre_index = cur_index; 
			}else{
				ROS_ERROR("what? cannot find cur_index > pre_index!");
				break; 
			}
		}

		// write gray image 
		{
      		cv_bridge::CvImage rgb_msg; 
      		// rgb_type = gray_img.type(); 
      		// if(rgb_type == CV_8UC1) // only check 8UC1, future 16UC1
			rgb_msg.encoding = sensor_msgs::image_encodings::MONO8; 
			// rgb_msg.encoding = sensor_msgs::image_encodings::BGR8; 
      		rgb_msg.header.stamp = rost;
      		rgb_msg.image = gray_img; 
      		cout <<"write rgb timestamp = "<<std::fixed<<rost.toSec()<<" string = "<<time_str<<endl;
      		bag.write(rgb_tpc, rost, rgb_msg); 
    	}

    	// write depth image 

    	{
			cv_bridge::CvImage dpt_msg; 
			// dpt_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; 
			dpt_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1; 
			dpt_msg.header.stamp = rost; 
			dpt_msg.image = dpt_img; 
			bag.write(dpt_tpc, rost, dpt_msg); 
    	}

		cv::imshow("intensity_image", gray_img); 
		cv::waitKey(5);

		++it; 
		usleep(10); 
	}
	bag.close(); 
	cout<<"sr4k_bdat_imu_to_bag: finish converting!"<<endl; 
	return ;
}


bool loadIMUFile(vector<double>& timestamp, vector<vector<double> >& vimus)
{
  ifstream inf(g_imu_file.c_str()); 
  
  if(!inf.is_open())
  {
    printf("%s failed to open imu file %s\n", __FILE__, g_imu_file.c_str()); 
    return false; 
  }
  double t; 
  vector<double> imu_data(6, 0.); 
  double ax, ay, az, gx, gy, gz, yaw, pitch, roll; 

  timestamp.clear(); 
  vimus.clear(); 
  while(!inf.eof())
  {
    
    inf>>t>>ax>>ay>>az>>gx>>gy>>gz>>yaw>>pitch>>roll; 
	imu_data[0] = gx; 
	imu_data[1] = gy; 
	imu_data[2] = gz; 
	imu_data[3] = ax; 
	imu_data[4] = ay; 
	imu_data[5] = az;     
	timestamp.push_back(t); 
	vimus.push_back(imu_data); 
  }
  printf("%s succeed to load %i imu measurements\n", __FILE__, timestamp.size()); 
  inf.close(); 
  return true; 


}

int find_index(vector<double>& imu_times, int pre_index, double cur_img_tp)
{
	int ret = pre_index; 
	for(int i = ret+1; i < imu_times.size(); i++){
		if(imu_times[i] >= cur_img_tp){
			ret = i-1; 
			break; 
		}
	}
	return ret; 
}

bool read_timestamp(string ts_file, map<string, string> & files)
{
	ifstream inf(ts_file.c_str()); 
	if(!inf.is_open()){
		cout <<" failed to load timestamp file: "<<ts_file<<endl; 
		return false; 
	}
	string s; 
	files.clear(); 
	while(!inf.eof()){
		getline(inf, s); 
		if(s.empty()) break; 
		stringstream ss; 
		string fname, timestamp; 
		ss << s; 
		ss >> fname >> timestamp; 
		files[fname] = timestamp; 
	}
	cout <<"sr4k_bdat_to_rgbd: load "<<files.size()<<" data"<<endl; 
	return true; 

}

bool loadImgTime(map<int, double>& mt) // load synchronized time stamps
{
  ifstream inf(g_img_time_file);
  if(!inf.is_open())
  {
    ROS_ERROR("%d failed to load camera timestamp %s", __LINE__, g_img_time_file.c_str()); 
    return false; 
  }
  int img_id; 
  double img_timestamp; 
  while(!inf.eof())
  { 
    inf>>img_id>>img_timestamp; 
    // ROS_INFO("img_id %d time %lf", img_id, img_timestamp);
    mt[img_id] = img_timestamp; 
  }
  return mt.size() > 0; 
}
