#include "map_image.h"

namespace zw {

using namespace cv;

MapImage::MapImage()
{

}

MapImage::~MapImage()
{

}

void MapImage::GetQImage(const cv::Mat& image,QImage &img)
{
    if(image.channels()==3){           
        //cvt Mat BGR 2 QImage RGB
        Mat rgb=image;
        cvtColor(image,rgb,CV_BGR2RGB);
        img =QImage((const unsigned char*)(rgb.data),rgb.cols,rgb.rows,
                    rgb.cols*rgb.channels(),QImage::Format_RGB888);
    }else{
        img =QImage((const unsigned char*)(image.data),image.cols,image.rows,
                    image.cols*image.channels(),QImage::Format_RGB888);
    }
}

}
