/*
 *  Jun. 9 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish rgb, dpt given folder like: 
 *
 *  timestamp.txt color/ depth/
 *  
 * */
#include "img_pub_rgbd.h"
#include <fstream>

using namespace std; 

CImgPubRGBD::CImgPubRGBD(){}
CImgPubRGBD::~CImgPubRGBD(){}

bool CImgPubRGBD::getData(std::string dir)
{
  string associationFile = dir + "/timestamp.txt"; 
  mb_data_ready = loadImages(associationFile); 
  m_data_dir = dir; 
  return mb_data_ready; 
}

bool CImgPubRGBD::loadImages(const std::string strAssociationFilename)
{
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  if(!fAssociation.is_open())
  {
    ROS_ERROR("img_pub_rgbd.cpp: failed to open file %s", strAssociationFilename.c_str()); 
    return false; 
  }
  
  // clear 
  mv_rgb.clear(); 
  mv_dpt.clear(); 
  mv_timestamp.clear(); 

  string s;
  getline(fAssociation,s); // delete the first line  

  while(!fAssociation.eof())
  {
    string s;
    getline(fAssociation,s);// 
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD;
      ss >> t;
      mv_timestamp.push_back(t);
      ss >> sRGB;
      mv_rgb.push_back(sRGB);
      ss >> t;
      ss >> sD;
      mv_dpt.push_back(sD);
    }
  }
  return true; 
}

bool CImgPubRGBD::currValid()
{
  if(mb_data_ready == false)
  {
    ROS_ERROR("img_pub_rgbd.cpp: data is not ready, call getData() first !"); 
    return false; 
  }
  if(m_curr_idx >= mv_rgb.size())
  {
    ROS_WARN("img_pub_rgbd.cpp: m_curr_idx = %d >= mv_rgb.size() %d", m_curr_idx, mv_rgb.size()); 
    return false; 
  }
  return true; 
}

bool CImgPubRGBD::getImg(string fname, cv::Mat& img)
{
  img = cv::imread(fname.c_str(), -1); 
  if(img.data == NULL)
  {
    ROS_WARN("img_pub_rgbd.cpp: failed to load img at %s", fname.c_str()); 
    return false; 
  }
  return true; 
}

bool CImgPubRGBD::getNextRGB(cv::Mat& rgb)
{
  if(!currValid()) return false;  
  string fname = m_data_dir + "/" + mv_rgb[m_curr_idx];
  return getImg(fname, rgb); 
}

bool CImgPubRGBD::getNextDpt(cv::Mat& dpt)
{
  if(!currValid()) return false; 
  string fname = m_data_dir + "/" + mv_dpt[m_curr_idx];
  return getImg(fname, dpt); 
}


