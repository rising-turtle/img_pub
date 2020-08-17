/*
 *  May 8 2019, He Zhang, hzhang8@vcu.edu 
 *
 *  decompress rosbag file from structure core, need to align them 
 *
 * */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std; 
using namespace cv; 

#define SQ(x) (((x)*(x)))

string base_dir(""); 

void processBagfile(string bagfile); 

void handle_rgb(const cv::Mat& rgb, cv::Mat& rgb_out);
void handle_dpt(const cv::Mat& dpt, cv::Mat& dpt_out);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bag_decompress_rgbd_imu_l515");
  
  ros::NodeHandle nh; 

  ROS_INFO("./bag_decompress_rgbd_imu_l515 [bagfile] [output_dir]");

  string bagfile = "";
  if(argc >= 2) 
    bagfile = argv[1]; 
  if(argc >= 3)
    base_dir = argv[2]; 

  processBagfile(bagfile); 

  return 0; 
}

void processBagfile(string bagfile)
{
  std::vector<std::string> topics;
  string rgb_tpc = "/camera/color/image_raw"; // "/cam0/image_raw";   
  string dpt_tpc = "/camera/aligned_depth_to_color/image_raw"; 
  // string ir_tpc  = "/cam0/ir";
  // string ir2_tpc = "/cam0/ir2"; 
  // string imu_tpc = "/imu0";
  topics.push_back(rgb_tpc); 
  topics.push_back(dpt_tpc); 
  // topics.push_back(ir_tpc);
  // topics.push_back(ir2_tpc);
  // topics.push_back(imu_tpc); 

  rosbag::Bag bag; 
  bag.open(bagfile, rosbag::bagmode::Read); 
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // mkdir 
  string d_dir = base_dir;//gDataName; 
  string d_rgb = d_dir + "/color"; 
  string d_dpt = d_dir + "/depth"; 
  // string d_ir = d_dir + "/ir"; 
  // string d_ir2 = d_dir + "/ir2"; 
  // std::cout<<d_dir<<"   d_dir_got|||"<<d_rgb<<"   d_rgb|||"<<d_dpt<<"   d_dpt|||"<<std::endl;

  mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_rgb.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_dpt.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir2.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  
  // imu file and timestamp file 
  // ofstream imu_f(d_dir + "/imu_vn100.log"); 
  ofstream img_f(d_dir + "/timestamp.txt");
  // img_f<<"index\ttimestamp"<<endl;

  // for extract cv::mat 
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;
  // cv_bridge::CvImageConstPtr cv_ptrIr; 
  // cv_bridge::CvImageConstPtr cv_ptrIr2; 

  string last_time(""); 

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    /*
    if(m.getTopic() == imu_tpc || ("/"+m.getTopic()) == imu_tpc)
    {
      // receive a imu message
      sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>(); 
      imu_f<<std::fixed<<simu->header.stamp<<"\t"<<simu->linear_acceleration.x << "\t"<<simu->linear_acceleration.y << "\t"
        <<simu->linear_acceleration.z<<"\t"<<simu->angular_velocity.x<<"\t"<<simu->angular_velocity.y<<"\t"
        <<simu->angular_velocity.z<<"\t"<<0.0<<"\t"<<0.0<<"\t"<<0.0<<endl;
      
      cout<<"bag_decompress.cpp: receive an IMU msg at time: "<<simu->header.stamp<<endl;

    }else // an image mssage 
    */
    if(!ros::ok()){
      break; 
    }

    {
      sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
      stringstream tt; 
      tt << simage->header.stamp;
      // cout << "bag_decompress.cpp: receive an image msg at time: "<<tt.str()<<endl; 
      cout << "bag_decompress_rgbd_structure_core.cpp: receive an image msg at time: "<<simage->header.stamp<<endl;

      if(last_time == "" || tt.str() != last_time) // first timestamp 
      {
        last_time = tt.str(); 
        // img_f<<last_time<<"\tcolor/"<<last_time<<".png"<<"\t"<<last_time<<"\tdepth/"<<last_time<<".exr"<<"\t"
        //  <<last_time<<"\tir/"<<last_time<<".png"<<"\t"<<last_time<<"\tir2/"<<last_time<<".png"<<endl;
        // img_f<<last_time<<"\tcolor/"<<last_time<<".png"<<"\t"<<last_time<<"\tdepth/"<<last_time<<".png"<<"\t"
        //  <<last_time<<"\tir/"<<last_time<<".png"<<"\t"<<last_time<<"\tir2/"<<last_time<<".png"<<endl;
        img_f<<last_time<<"\tcolor/"<<last_time<<".png"<<"\t"<<last_time<<"\tdepth/"<<last_time<<".png"<<endl;
      }
      
      if(m.getTopic() == rgb_tpc || ("/"+m.getTopic()) == rgb_tpc)
      {
        // receive a rgb image 
        // cv_ptrRGB = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::BGR8); 
        cv_ptrRGB = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_8UC3); 

        // handle rgb 
        cv::Mat rgb; 
        handle_rgb(cv_ptrRGB->image, rgb); 
        // imwrite(d_rgb + "/"+ tt.str()+".png", cv_ptrRGB->image); 
	      imshow("rgb_file", rgb); 
        imwrite(d_rgb + "/"+ tt.str()+".png", rgb); 
	      waitKey(3);
      }
      if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc)
      {
        // receive a dpt image
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1); 

        // handle dpt 
        cv::Mat dpt;
        handle_dpt(cv_ptrD->image, dpt); 
        // imwrite(d_dpt + "/"+tt.str() +".png", cv_ptrD->image); 
        imshow("dpt_file", cv_ptrD->image); 
        imwrite(d_dpt + "/"+tt.str() +".png", dpt); 
        waitKey(3); 
      }
    }
  }
  return ; 
}


void handle_rgb(const cv::Mat& rgb, cv::Mat& rgb_out)
{
  // undistortion 

  float dist_data[5] = {0.14949573576450348, -0.49464672803878784, 0.00035660676076076925, 0.0001328527432633564, 0.42560458183288574}; 
  cv::Mat distCoeffs(1, 5, CV_32F, dist_data);  
  float cam_matrix_data[9] = {907.6046142578125, 0.0, 648.9068603515625, 0.0, 908.1854858398438, 371.2373352050781, 0.0, 0.0, 1.0};  
  cv::Mat cameraMatrix(3, 3, CV_32F, cam_matrix_data); 

  cv::undistort(rgb, rgb_out, cameraMatrix, distCoeffs); 

  // ROS_DEBUG("align_struct_core: rgb_out size %d x %d", rgb_out.cols, rgb_out.rows); 
}

void handle_dpt(const cv::Mat& dpt, cv::Mat& dpt_out)
{
  // construct 3d points 
  vector<Point3f> pts;
  pts.reserve(dpt.rows * dpt.cols);  
  float fx, fy, cx, cy; // not distortion for depth camera 

  // depth camera
  fx = 907.6046142578125; fy = 556.875; cx = 295.5; cy = 232.25; 
  float z,x,y ; 
  for(int r = 0; r<dpt.rows; r++){
    for(int c = 0; c<dpt.cols; c++){
      z = dpt.at<unsigned short>(r,c) * 0.001; 
      if(z >= 0.3 && z <= 7){ // range of structure core
        x = ((c - cx)/fx) * z; 
        y = ((r - cy)/fy) * z; 
        pts.push_back(Point3f(x, y, z)); 
      }
    }
  }

  // transform into color reference 
  Eigen::Matrix4f Tc2d; 
  Tc2d << 0.9999665021896362, 0.008139053359627724, 0.0008413196774199605, -0.0013251318596303463,
    -0.008115333504974842, 0.9996509552001953, -0.02514067105948925, 0.013551371172070503,
    -0.0010456473100930452, 0.025133000686764717, 0.9996835589408875, -0.004039745312184095,
    0, 0, 0, 1;

  // Eigen::Matrix4f Tc2d = Td2c.inverse(); 
  for(int i=0; i<pts.size(); i++){
    Point3f& pt = pts[i]; 
    Eigen::Vector4f pt_d(pt.x, pt.y, pt.z, 1.0); 
    Eigen::Vector4f pt_c = Tc2d * pt_d; 
    pt = Point3f(pt_c(0), pt_c(1), pt_c(2)); 
  }
  vector<Point2f> pts_2d(pts.size()); 

  // color camera 
   fx = 444.277; 
   fy = 444.764; 
   cx = 324.055; 
   cy = 254.516; 

  // (u - cx)/x = fx/z

  for(int i=0; i<pts.size(); i++){
    Point2f& pt_2d = pts_2d[i];
    Point3f& pt = pts[i];  
    pt_2d.x = (pt.x/pt.z)*fx + cx; 
    pt_2d.y = (pt.y/pt.z)*fy + cy; 
  }

  // ROS_INFO("point 3d %d point 2d : %d", pts.size(), pts_2d.size());
/*
  // project pts into image 
  float cam_matrix_data[9] = {444.277, 0, 324.055, 0, 444.764, 254.516, 0, 0, 1};  
  cv::Mat cameraMatrix(3, 3, CV_32F, cam_matrix_data); 
  float dist_data[5] = {0,0,0,0,0}; //{0.320760, -0.864822, 0, 0, 0.589437}; 
  cv::Mat distCoeffs(1, 5, CV_32F, dist_data);  
  float rvec_data[3] = {0,0,0}; 
  cv::Mat rvec(3,1, CV_32F, rvec_data); 
  float tvec_data[3] = {0,0,0}; 
  cv::Mat tvec(3,1, CV_32F, tvec_data); 

  projectPoints(pts, rvec, tvec, cameraMatrix, distCoeffs, pts_2d);
  ROS_DEBUG("after projectPoints");
*/
  // 
  cv::Mat dpt_dis = cv::Mat(dpt.rows, dpt.cols, CV_32FC1, Scalar(0.0)); 
  // cv::Mat dpt_cnt = cv::Mat(dpt.rows, dpt.cols, CV_32FC1, Scalar(0.0)); 
  // out_dpt = cv::Mat(dpt.cols, dpt.rows, CV_16UC1, Scalar) 
  int cnt = 0; 

  for(int i=0; i<pts_2d.size(); i++){
    // float c = pts_2d[i].x; 
    // float r = pts_2d[i].y; 
    
    int c = std::round(pts_2d[i].x); 
    int r = std::round(pts_2d[i].y); 

    if(c < 0 || r < 0 || c >= dpt.cols - 1 || r >= dpt.rows - 1)
      continue; 

    ++cnt; 

    dpt_dis.at<float>(r, c) = pts[i].z; 

 }

  // ROS_INFO("valid points number: %d", cnt);

  // convert 
  dpt_dis.convertTo(dpt_out, CV_16UC1, 1000);
  return ; 
}
