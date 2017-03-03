#ifndef MAP_IMAGE_H
#define MAP_IMAGE_H

#include <qpixmap.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>


namespace zw {

class MapImage
{
public:
    int filter_cnt;
    int sample_cnt;


public:
    MapImage();
    ~MapImage();
    void GetBinaryImage(const cv::Mat& imgIn,double th,cv::Mat& imgOut,cv::Mat& sample_imgOut );
    void GetQImage(const cv::Mat& image,QImage &img);
    void SurfFeatureMatch(const cv::Mat& map,const cv::Mat &subMap);
    void OrbFeaturematch(const cv::Mat& map,const cv::Mat &subMap);
    void SiftFeaturematch(const cv::Mat& map,const cv::Mat &subMap);

    void ShowBinaryImage(const std::string& winname,const cv::Mat& imgIn,double th);

 //   void

};

}



#endif // MAP_IMAGE_H
