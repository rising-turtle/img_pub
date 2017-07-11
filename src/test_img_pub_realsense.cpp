/*
 *  Jun. 7 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  test publish realsense image data  
 *
 * */

#include <ros/ros.h>
#include "img_pub_realsense.h"
#include <string>

using namespace std; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_img_pub_realsense"); 
  ros::NodeHandle n; 
  ros::NodeHandle np("~"); 
  string data_dir("/media/davidz/work/work/data/realsense/etasF4_06_01_01"); 
  np.param("data_dir", data_dir, data_dir); 

  if(argc >= 2)
    data_dir = string(argv[1]); 

  ROS_WARN("pub_realsense.cpp: publish dataset in %s", data_dir.c_str());

  CImgPubRS pub; 
  pub.getData(data_dir); 

  while(ros::ok())
  {
    if(!pub.publishRGBD_IR2())
    {
      break; 
    }
    ROS_INFO("test_img_pub_realsense.cpp: succeed to publish a new image!"); 
  }

  return 1; 
}





