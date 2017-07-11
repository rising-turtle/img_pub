/*
 *  Jun. 7 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish rgb, dpt, ir, ir2 given folder like: 
 *
 *  timestamp.txt color/ depth/ ir/ ir2/
 *  
 * */

#include "img_pub.h"
#include <vector>
#include <string>

class CImgPubRS : public CImgPub
{
  public:
    CImgPubRS(); 
    virtual ~CImgPubRS(); 
    
    virtual bool getData(std::string dir); 
    virtual bool getNextRGB(cv::Mat& );
    virtual bool getNextDpt(cv::Mat& ); 
    virtual bool getNextIr1(cv::Mat& );
    virtual bool getNextIr2(cv::Mat& );
    virtual bool getNextTimeStamp(std::string& time); 

    // get data
    bool loadImages(const std::string strAssociationFilename);
    bool currValid(); // check whether the current idx is valid 
    bool getImg(std::string fname, cv::Mat& img); 

    std::vector<std::string> mv_rgb;
    std::vector<std::string> mv_dpt;
    std::vector<std::string> mv_ir1;
    std::vector<std::string> mv_ir2; 
    // std::vector<double>      mv_timestamp; 
    std::vector<std::string> mv_timestamp; 
};


