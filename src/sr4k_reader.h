/*
	Api. 6 2019, He Zhang, hzhang8@vcu.edu 
	
	read sr4000 data, covert it to opencv format 

*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std; 
typedef float SR_TYPE; 
typedef unsigned short SR_IMG_TYPE;
const static int SR_WIDTH = 176; 
const static int SR_HEIGHT = 144; 
const static int SR_SIZE = SR_WIDTH*SR_HEIGHT;

class sr_data
{
  public:
    sr_data(){}
  public:
    char mono_intensity_[SR_SIZE];
    SR_IMG_TYPE intensity_[SR_SIZE];
  union{
    struct
    {
      SR_IMG_TYPE dis_[SR_SIZE];
      // unsigned char cam_handle_[SR_SIZE]; // for buffering the cam_handle     
    };
    struct
    {
      SR_TYPE z_[SR_SIZE]; 
      SR_TYPE x_[SR_SIZE]; 
      SR_TYPE y_[SR_SIZE]; 
      SR_IMG_TYPE c_[SR_SIZE]; 
    };
  };
  double timestamp_;   // 
  float gt_pv_[7];     // ground truth pose: x y z qx qy qz qw
  bool b_gt_has_been_set_ ; 
  static const int _sr_size = SR_SIZE; 
  static const int _sr_width = SR_WIDTH; 
  static const int _sr_height = SR_HEIGHT;
  typedef SR_IMG_TYPE _sr_type;
};

class SReader{

public:
	SReader(); 
	~SReader(); 
	bool readBdatFrame(string f_name, sr_data& d); 
	bool readBdatFrame(string f_name, cv::Mat& gray_img, cv::Mat& dpt_img); 
	void fromSRToCV(sr_data& d, cv::Mat& gray_img, cv::Mat& dpt_img); 
};