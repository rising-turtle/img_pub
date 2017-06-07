/*
 *  Jun. 6 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  test publish sequential image data  
 *
 * */

#include <ros/ros.h>
#include "img_pub_seq.h"
#include <string>

using namespace std; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_img_pub_seq"); 
  ros::NodeHandle n; 
  ros::NodeHandle np("~"); 
  string data_dir("/home/davidz/work/data/sin2_tex2_h1_v8_d/img"); 
  np.param("data_dir", data_dir, data_dir); 

  CImgPubSeq pub; 
  pub.getData(data_dir); 

  while(ros::ok())
  {
    if(!pub.publishRGB())
    {
      break; 
    }
    ROS_INFO("test_img_pub_seq.cpp: succeed to publish a new image!"); 
  }

  return 1; 
}





