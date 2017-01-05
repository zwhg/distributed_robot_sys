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
    MapImage();
    ~MapImage();
    void GetBinaryImage(const cv::Mat& imgIn,cv::Mat& imgOut);
    void GetQImage(const cv::Mat& image,QImage &img);
    void SurfFeatureMatch(const cv::Mat& map,const cv::Mat &subMap);

};

}



#endif // MAP_IMAGE_H
