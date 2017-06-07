/*
 *  Jun. 6 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish img with sequence images with the following format
 *  
 *  [prefix]_[0*]id[suffix]
 *
 * */

#include "img_pub_seq.h"
#include <sstream>

using namespace std;  

CImgPubSeq::CImgPubSeq():
  CImgPub()
{
  ros::NodeHandle np("~"); 
  m_prefix = "frame_"; 
  m_suffix = "_0.png"; 
  m_start_id = 2; 
  m_stop_id = -1; 
  m_padding_len = 6; 
  np.param("prefix", m_prefix, m_prefix);
  np.param("suffix", m_suffix, m_suffix); 
  np.param("start_id", m_start_id, m_start_id); 
  np.param("stop_id", m_stop_id, m_stop_id); 
  np.param("padding_len", m_padding_len, m_padding_len); 

}

CImgPubSeq::~CImgPubSeq(){}

bool CImgPubSeq::getData(std::string dir)
{
  m_data_dir = dir; 
  m_curr_idx = m_start_id; 
  mb_data_ready = true; 
  return true; 
}

bool CImgPubSeq::getNextRGB(cv::Mat& rgb)
{
  if(mb_data_ready == false)
  {
    ROS_ERROR("img_pub_seq.cpp: data is not ready, call getData() first!"); 
    return false; 
  }

  if(m_stop_id > 0 && m_curr_idx >= m_stop_id)
  {
    ROS_WARN("img_pub_seq.cpp: m_curr_idx %d >= m_stop_id %d", m_curr_idx, m_stop_id); 
    return false; 
  }

  stringstream ss; 
  ss <<m_data_dir<<"/"<< m_prefix<<setfill('0')<<setw(m_padding_len)<<m_curr_idx++<<m_suffix; 

  rgb = cv::imread(ss.str().c_str(), -1); 

  if(rgb.data == NULL)
  {
    ROS_WARN("img_pub_seq.cpp: failed to load img at %s", ss.str().c_str()); 
    return false; 
  }
  return true; 
}

