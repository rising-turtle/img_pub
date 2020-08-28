/*
 *  Aug. 28 2020, He Zhang, hzhang8@vcu.edu 
 *
 *  convert the original bag into one used by vio  
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

string bag_out(""); 

struct v3d{

  double data[3]; 
  ros::Time timestamp;  

}; 

class syn_imu{

public:
  double linearInterpolate(double t, const double lhs, const double rhs){
    return lhs + t * (rhs - lhs);
  }

  void newAccSample(const sensor_msgs::ImuConstPtr& simu){
    v3d d; 
    d.timestamp = simu->header.stamp; 
    d.data[0] = simu->linear_acceleration.x; 
    d.data[1] = simu->linear_acceleration.y; 
    d.data[2] = simu->linear_acceleration.z; 
    // accel_events[x.timestamp()] = x;    
    // num_acc = accel_events.size();
    acc_buf.push_back(d); 
  }

  void newGyroSample(const sensor_msgs::ImuConstPtr& simu){
    v3d d; 
    d.timestamp = simu->header.stamp; 
    d.data[0] = simu->angular_velocity.x; 
    d.data[1] = simu->angular_velocity.y;
    d.data[2] = simu->angular_velocity.z; 
    // gyro_events[x.timestamp()] = x;
    // num_gyro = gyro_events.size();
    gyo_buf.push_back(d); 
  }

  vector<sensor_msgs::Imu> getIMU(){
    vector<sensor_msgs::Imu> m; 
    while(true){
      if(acc_buf.empty() || gyo_buf.empty()) 
        return m; 
      if(acc_buf.size()<= 1)
        return m; 

      double acc_t = acc_buf.front().timestamp.toSec(); 
      double gyo_t = gyo_buf.front().timestamp.toSec(); 
      double t_diff = gyo_t - acc_t; //fabs(acc_t - gyo_t); 

      if(t_diff == 0){
          sensor_msgs::Imu data; 
          data.angular_velocity.x = gyo_buf.front().data[0]; 
          data.angular_velocity.y = gyo_buf.front().data[1];
          data.angular_velocity.z = gyo_buf.front().data[2];

          data.linear_acceleration.x = acc_buf.front().data[0]; 
          data.linear_acceleration.y = acc_buf.front().data[1]; 
          data.linear_acceleration.z = acc_buf.front().data[2];

          m.emplace_back(data); 
          acc_buf.pop_front(); 
          gyo_buf.pop_front();
          continue; 
      }

      if(t_diff > 0){
          // find the closest 
          v3d pre_acc; // acc on the left side of gyo
          double acc_nt = acc_t; 
          while(1){
              pre_acc = acc_buf.front(); 
              acc_t = pre_acc.timestamp.toSec(); 
              acc_buf.pop_front(); 
              if(acc_buf.empty()) 
                  break; 
              acc_nt = acc_buf.front().timestamp.toSec(); 
              if(acc_nt >= gyo_t){
                  acc_buf.push_front(pre_acc);
                  break; 
              }
          }

          if(acc_nt < gyo_t){ // cannot find a acc on the right side of gyo
              acc_buf.push_front(pre_acc); 
              continue; 
          }

          sensor_msgs::Imu data; 
          data.header.stamp = gyo_buf.front().timestamp; 
          data.header.frame_id = "/imu"; 
          data.angular_velocity.x = gyo_buf.front().data[0]; 
          data.angular_velocity.y = gyo_buf.front().data[1];
          data.angular_velocity.z = gyo_buf.front().data[2];
          {
              double ax = pre_acc.data[0]; 
              double ay = pre_acc.data[1]; 
              double az = pre_acc.data[2];
              
              // double acc_nt = acc_buf.front().t.toSec(); 
              t_diff = gyo_t - acc_t; 
              double ratio = t_diff/(acc_nt - acc_t); 

              data.linear_acceleration.x = ax + ratio*(acc_buf.front().data[0] - ax); 
              data.linear_acceleration.y = ay + ratio*(acc_buf.front().data[1] - ay); 
              data.linear_acceleration.z = az + ratio*(acc_buf.front().data[2] - az); 

              cout <<std::fixed<<"ratio: "<<ratio<<" t_diff: "<<t_diff<<" data_timestamp: "<<data.header.stamp.toSec()<<" gyo_t: "<<gyo_t<<" acc_t: "<<acc_t<<" acc_nt: "<<acc_nt<<endl;

              m.emplace_back(data); 
              // acc_buf.pop();
              gyo_buf.pop_front();
              continue; 
          }
      }else{ // e. gyo_t 1s, acc_t 1.01s, no good 
          gyo_buf.pop_front(); 
          continue; 
      }

    }
  }

  // queue<v3d> gyo_buf; 
  // queue<v3d> acc_buf; 

  deque<v3d> gyo_buf; 
  deque<v3d> acc_buf; 

};

void processBagfile(string bagfile); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bag_conversion_l515");
  
  ros::NodeHandle nh; 

  ROS_INFO("./bag_conversion_l515 [in_bagfile] [out_bagfile]");

  string bagfile = "";
  if(argc >= 2) 
    bagfile = argv[1]; 
  if(argc >= 3)
    bag_out = argv[2]; 

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
  string acc_tpc = "/camera/accel/sample";
  string gyo_tpc = "/camera/gyro/sample"; 
  topics.push_back(rgb_tpc); 
  topics.push_back(dpt_tpc); 
  // topics.push_back(ir_tpc);
  // topics.push_back(ir2_tpc);
  topics.push_back(gyo_tpc); 
  topics.push_back(acc_tpc); 

  string imu_tpc = "/imu"; 

  rosbag::Bag bag; 
  bag.open(bagfile, rosbag::bagmode::Read); 
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // rosbag for write 
  rosbag::Bag outbag; 
  outbag.open(bag_out, rosbag::bagmode::Write); 

  // for extract cv::mat 
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;
  // cv_bridge::CvImageConstPtr cv_ptrIr; 
  // cv_bridge::CvImageConstPtr cv_ptrIr2; 

  string last_time(""); 

  syn_imu syn; 

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if(!ros::ok()){
      break; 
    }

    if(m.getTopic() == gyo_tpc){
        // receive a imu message
        sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>(); 
        cout<<"bag_conversion_l515.cpp: receive an gyro at time: "<<simu->header.stamp<<endl;
        syn.newGyroSample(simu); 

        vector<sensor_msgs::Imu> mv = syn.getIMU(); 
        for(int i=0; i<mv.size(); i++){
          outbag.write(imu_tpc, mv[i].header.stamp, mv[i]); 
        }
      }
    else if(m.getTopic() == acc_tpc){
          // receive a imu message
        sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>(); 
        cout<<"bag_conversion_l515.cpp: receive an accel at time: "<<simu->header.stamp<<endl;
        syn.newAccSample(simu);

        vector<sensor_msgs::Imu> mv = syn.getIMU(); 
        for(int i=0; i<mv.size(); i++){
          outbag.write(imu_tpc, mv[i].header.stamp, mv[i]); 
        }
      }
    else
    {
      sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
      stringstream tt; 
      tt << simage->header.stamp;
      // cout << "bag_decompress.cpp: receive an image msg at time: "<<tt.str()<<endl; 
      cout << "bag_conversion_l515.cpp: receive an image msg at time: "<<simage->header.stamp<<endl;

      if(last_time == "" || tt.str() != last_time) // first timestamp 
      {
        last_time = tt.str(); 
        // img_f<<last_time<<"\tcolor/"<<last_time<<".png"<<"\t"<<last_time<<"\tdepth/"<<last_time<<".exr"<<"\t"
        //  <<last_time<<"\tir/"<<last_time<<".png"<<"\t"<<last_time<<"\tir2/"<<last_time<<".png"<<endl;
        // img_f<<last_time<<"\tcolor/"<<last_time<<".png"<<"\t"<<last_time<<"\tdepth/"<<last_time<<".png"<<"\t"
        //  <<last_time<<"\tir/"<<last_time<<".png"<<"\t"<<last_time<<"\tir2/"<<last_time<<".png"<<endl;
        // img_f<<last_time<<"\tcolor/"<<last_time<<".png"<<"\t"<<last_time<<"\tdepth/"<<last_time<<".png"<<endl;
      }
      
      if(m.getTopic() == rgb_tpc || ("/"+m.getTopic()) == rgb_tpc)
      {
        // receive a rgb image 
        // cv_ptrRGB = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::BGR8); 
        cv_ptrRGB = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_8UC3); 
        // handle rgb 
        cv::Mat rgb; 
        // handle_rgb(cv_ptrRGB->image, rgb); 
        rgb = cv_ptrRGB->image; 
        outbag.write(rgb_tpc, simage->header.stamp, simage); 
	      imshow("rgb_file", rgb); 
	      waitKey(3);
      }
      if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc)
      {
        // receive a dpt image
        // cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1); 
        // handle dpt 
        // cv::Mat dpt;
        // handle_dpt(cv_ptrD->image, dpt); 
        // dpt = cv_ptrD->image; 
        // imwrite(d_dpt + "/"+tt.str() +".png", cv_ptrD->image); 
        // imshow("dpt_file", cv_ptrD->image); 
        // imwrite(d_dpt + "/"+tt.str() +".png", dpt); 
        outbag.write(dpt_tpc, simage->header.stamp, simage); 
        waitKey(3); 
      }
    }
  }
  outbag.close();
  return ; 
}
