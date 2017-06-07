/*
 *  Jun. 6 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  publish img with sequence images with the following format
 *  
 *  [prefix][0*]id[suffix]
 *
 * */

#include "img_pub.h"

class CImgPubSeq  : public CImgPub
{
  public:
    CImgPubSeq(); 
    virtual ~CImgPubSeq(); 

    virtual bool getData(std::string dir); 
    virtual bool getNextRGB(cv::Mat& ); 

    std::string m_prefix;   // prefix of image name
    std::string m_suffix;   // suffix of image name
    int m_start_id;         // start id 
    int m_stop_id;          // stop id
    int m_padding_len;      // number of 0s for padding 
};








