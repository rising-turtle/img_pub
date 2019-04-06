/*
	Api. 6 2019, He Zhang, hzhang8@vcu.edu 
	
	read sr4000 data, covert it to opencv format 

*/

#include "sr4k_reader.h"
#include <stdio.h>
#include <stdlib.h>

namespace{
void map_raw_img_to_grey(unsigned short * pRaw, unsigned char* pGrey, int N)
{
  unsigned short limit_s = 65000;
  unsigned short* p1 = pRaw; 
  unsigned char* p2 = pGrey; 
  
  unsigned char max_c = 0; 
  unsigned char tc = 0; 
  static vector<float> sqrt_map_;
  
  double scale = 255./65000.;

  if(sqrt_map_.size() <= 0)
  {
    int N = 65535; 
    sqrt_map_.resize(N);
    for(int i=0; i<N; i++)
      sqrt_map_[i] = (unsigned char)(sqrt(double(i))); 
  }
  
  for(int i=0; i<N; i++)
  {
    if(*p1 >= limit_s) // delete the values larger than 65000
      tc = 0; 
    else 
      tc = sqrt_map_[*p1];
    if(tc > max_c) {max_c = tc; }
    *p2 = tc;
    ++p1; 
    ++p2; 
  }
  assert(max_c > 0);
  p2 = pGrey;
  float inv_max = (float)(255.0/max_c);
  for(int i=0; i<N; i++)
  {
    *p2 = (unsigned char)((*p2)*inv_max);
    // *p2 = *p2*scale;
    ++p2;
  }
}
}


SReader::SReader(){}
SReader::~SReader(){}

bool SReader::readBdatFrame(string f_name, sr_data& d)
{
	FILE* fid = fopen(f_name.c_str(), "rb"); 
  	if(fid == NULL)
  	{
    	cout<<"SRreader: failed to open file: "<<f_name<<endl; 
    	return false;
  	}
  	 // old file version 
    fread(&d.z_[0], sizeof(SR_TYPE), SR_SIZE, fid); 
    fread(&d.x_[0], sizeof(SR_TYPE), SR_SIZE, fid);
    fread(&d.y_[0], sizeof(SR_TYPE), SR_SIZE, fid); 
    fread(&d.intensity_[0], sizeof(SR_IMG_TYPE), SR_SIZE, fid); 
    fread(&d.c_[0], sizeof(SR_IMG_TYPE), SR_SIZE, fid);
   	fclose(fid); 
    return true; 
}

void SReader::fromSRToCV(sr_data& sr, cv::Mat& i_img, cv::Mat& d_img)
{
	  // generate imgs and dpts 
  	cv::Mat cv_d_img, cv_i_img; 

	static vector<unsigned char> gray_img(SR_SIZE); 
	map_raw_img_to_grey(&sr.intensity_[0], &gray_img[0], SR_SIZE); 
	// map_raw_img_to_grey_linear(&sr.intensity_[0], &grey_img[0], SR_SIZE); 

	// cv::Mat raw_img = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_16UC1, &sr.intensity_[0], SR_WIDTH*sizeof(unsigned short)); 
	// cv_i_img = cv::Mat(raw_img.size(), CV_8UC1); 
	// raw_img.convertTo(cv_i_img, CV_8UC1, 0.05, -25); 

	cv_i_img = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_8UC1, &gray_img[0], SR_WIDTH*sizeof(unsigned char)); 

	static SR_IMG_TYPE dis_s[SR_SIZE] = {0}; 
	for(int i=0; i<SR_SIZE; ++i)
	{
		dis_s[i] = (SR_IMG_TYPE)(sr.z_[i]*1000);
	}
	cv_d_img = cv::Mat(SR_HEIGHT, SR_WIDTH, CV_16UC1, dis_s); 

  	i_img = cv_i_img.clone(); 
  	d_img = cv_d_img.clone();
  	return; 
}

bool SReader::readBdatFrame(string f_name, cv::Mat& gray_img, cv::Mat& dpt_img)
{
	sr_data d;
	if(!readBdatFrame(f_name, d)) return false; 
	fromSRToCV(d, gray_img, dpt_img); 
	return true; 
}