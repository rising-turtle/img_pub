/*
 *  Jun. 9 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish rgb, dpt given folder like: 
 *
 *  timestamp.txt color/ depth/
 *  
 * */

#include "img_pub.h"
#include <vector>
#include <string>

class CImgPubRGBD : public CImgPub
{
  public:
    CImgPubRGBD(); 
    virtual ~CImgPubRGBD(); 
    
    virtual bool getData(std::string dir); 
    virtual bool getNextRGB(cv::Mat& );
    virtual bool getNextDpt(cv::Mat& ); 

    // get data
    bool loadImages(const std::string strAssociationFilename);
    bool currValid(); // check whether the current idx is valid 
    bool getImg(std::string fname, cv::Mat& img); 

    std::vector<std::string> mv_rgb;
    std::vector<std::string> mv_dpt;
    std::vector<double>      mv_timestamp; 
};


