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
    void GetQImage(const cv::Mat& image,QImage &img);

};

}



#endif // MAP_IMAGE_H
