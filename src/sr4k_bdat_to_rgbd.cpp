/*
	Apr. 6 2019, He Zhang, hzhang8@vcu.edu 
	
	convert the sr4000's bdat dataset to the rgbd format 

*/

#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <iomanip>
#include <sys/stat.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
// #include <filesystem> 
#include "sr4k_reader.h"

using namespace std; 

// namespace fs=std::filesystem; 

string sr4k_folder(""); 
string rgbd_folder(""); 

void do_it(); 

int main(int argc, char* argv[])
{
	if(argc < 3){
		cout<<" usage: ./sr4k_bdat_to_rgbd [sr4k_folder] [new_folder] "<<endl; 
		return -1; 
	}

	sr4k_folder = argv[1]; 
	rgbd_folder = argv[2]; 

	do_it(); 

	return 0; 
}

bool read_timestamp(string ts_file, map<string, string> & files); 

void do_it()
{
	string timestamp_file = sr4k_folder + "/timestamp.log"; 
	map<string, string> files; 

	if(!read_timestamp(timestamp_file, files)) return; 
	cout <<"sr4k_bdat_to_rgbd: start to convert!"<<endl; 

	// mkdirs 
	string d_rgb = rgbd_folder + "/color"; 
	string d_dpt = rgbd_folder + "/depth"; 
	mkdir(rgbd_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  	mkdir(d_rgb.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  	mkdir(d_dpt.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	ofstream d_timestamp(rgbd_folder+"/timestamp.txt"); 
	d_timestamp << "index\ttimestamp"<<endl; 

	// try to copy imu 
	// string imu_from = sr4k_folder + "/imu_v100.log"; 
	// string imu_to = rgbd_folder + "/imu_v100.log"; 
	// fs::path p1 = imu_from.c_str(); 
	// fs::path p2 = imu_to.c_str(); 
	// fs::copy_file(imu_from.c_str(), imu_to.c_str());

	// try to convert 
	SReader sr4k; 
	map<string, string>::iterator it = files.begin(); 

	while(it!= files.end()){

		string bdat_file = sr4k_folder + "/d1_"+it->first+".bdat"; 

		// read file 
		cv::Mat gray_img, dpt_img; 
		if(!sr4k.readBdatFrame(bdat_file, gray_img, dpt_img)) break; 

		string time_str = it->second; 
		string rgb_str = "color/"+time_str+".png"; 
		string dpt_str = "depth/"+time_str+".png"; 
		d_timestamp<<time_str<<"\t"<<rgb_str<<"\t"<<time_str<<"\t"<<dpt_str<<endl; 

		rgb_str = rgbd_folder+"/"+rgb_str;
		dpt_str = rgbd_folder+"/"+dpt_str;

		cv::imwrite(rgb_str.c_str(), gray_img); 
		cv::imwrite(dpt_str.c_str(), dpt_img); 
		cv::imshow("intensity_image", gray_img); 
		cv::waitKey(5);

		++it; 
	}
	cout<<"sr4k_bdat_to_rgbd: finish converting!"<<endl; 
	return ;
}

bool read_timestamp(string ts_file, map<string, string> & files){
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