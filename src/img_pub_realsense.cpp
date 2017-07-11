/*
 *  Jun. 7 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish rgb, dpt, ir, ir2 given folder like: 
 *
 *  timestamp.txt color/ depth/ ir/ ir2/
 *  
 * */
#include "img_pub_realsense.h"
#include <fstream>

using namespace std; 

CImgPubRS::CImgPubRS(){}
CImgPubRS::~CImgPubRS(){}

bool CImgPubRS::getData(std::string dir)
{
  string associationFile = dir + "/timestamp.txt"; 
  mb_data_ready = loadImages(associationFile); 
  m_data_dir = dir; 
  return mb_data_ready; 
}

bool CImgPubRS::loadImages(const std::string strAssociationFilename)
{
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  if(!fAssociation.is_open())
  {
    ROS_ERROR("img_pub_realsense.cpp: failed to open file %s", strAssociationFilename.c_str()); 
    return false; 
  }
  
  // clear 
  mv_rgb.clear(); 
  mv_dpt.clear(); 
  mv_ir1.clear(); 
  mv_ir2.clear(); 
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
      // double t;
      string t; 
      string sRGB, sD, sr1, sr2;
      ss >> t;
      mv_timestamp.push_back(t);
      ss >> sRGB;
      mv_rgb.push_back(sRGB);
      ss >> t;
      ss >> sD;
      mv_dpt.push_back(sD);
      ss >> t; 
      ss >> sr1; 
      mv_ir1.push_back(sr1); 
      ss >> t; 
      ss >> sr2; 
      mv_ir2.push_back(sr2); 
    }
  }
  return true; 
}

bool CImgPubRS::currValid()
{
  if(mb_data_ready == false)
  {
    ROS_ERROR("img_pub_realsense.cpp: data is not ready, call getData() first !"); 
    return false; 
  }
  if(m_curr_idx >= mv_rgb.size())
  {
    ROS_WARN("img_pub_realsense.cpp: m_curr_idx = %d >= mv_rgb.size() %d", m_curr_idx, mv_rgb.size()); 
    return false; 
  }
  return true; 
}

bool CImgPubRS::getImg(string fname, cv::Mat& img)
{
  img = cv::imread(fname.c_str(), -1); 
  if(img.data == NULL)
  {
    ROS_WARN("img_pub_realsense.cpp: failed to load img at %s", fname.c_str()); 
    return false; 
  }
  return true; 
}

bool CImgPubRS::getNextTimeStamp(std::string& t)
{
  if(!currValid()) return false; 
  t = mv_timestamp[m_curr_idx];
  return true; 
}

bool CImgPubRS::getNextRGB(cv::Mat& rgb)
{
  if(!currValid()) return false;  
  string fname = m_data_dir + "/" + mv_rgb[m_curr_idx];
  return getImg(fname, rgb); 
}

bool CImgPubRS::getNextDpt(cv::Mat& dpt)
{
  if(!currValid()) return false; 
  string fname = m_data_dir + "/" + mv_dpt[m_curr_idx];
  return getImg(fname, dpt); 
}

bool CImgPubRS::getNextIr1(cv::Mat& ir)
{
  if(!currValid()) return false; 
  string fname = m_data_dir + "/" + mv_ir1[m_curr_idx]; 
  return getImg(fname, ir);
}

bool CImgPubRS::getNextIr2(cv::Mat& ir)
{
  if(!currValid()) return false; 
  string fname = m_data_dir + "/" + mv_ir2[m_curr_idx]; 
  return getImg(fname, ir);
}




