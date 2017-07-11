/*
 *  Jun. 9 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  test publish rgbd dataset  
 *
 * */

#include <ros/ros.h>
#include "img_pub_rgbd.h"
#include <string>

using namespace std; 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_img_pub_rgbd"); 
  ros::NodeHandle n; 
  ros::NodeHandle np("~"); 
  string data_dir("/media/davidz/work/work/data/realsense/Q_desk_loop"); 
  np.param("data_dir", data_dir, data_dir); 

  if(argc >= 2)
    data_dir = string(argv[1]); 

  ROS_WARN("pub_rgbd.cpp: publish dataset in %s", data_dir.c_str());

  CImgPubRGBD pub; 
  pub.getData(data_dir); 

  while(ros::ok())
  {
    if(!pub.publishRGBD())
    {
      break; 
    }
    ROS_INFO("test_img_pub_rgbd.cpp: succeed to publish a new image!"); 
  }

  return 1; 
}





