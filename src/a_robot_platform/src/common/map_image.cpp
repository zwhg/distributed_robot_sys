#include "map_image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <qdebug.h>

namespace zw {

using namespace cv;
using namespace std;

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

void MapImage::SurfFeatureMatch(const cv::Mat& map,const cv::Mat &subMap)
{
    if(map.empty() ||subMap.empty())
    {
       qDebug()<< "please open two map image!";
       return;
    }

    Mat image1 =map.clone();
    Mat image2 =subMap.clone();

    //转化为灰度图
    cvtColor(image1,image1,CV_BGR2GRAY);
    cvtColor(image2,image2,CV_BGR2GRAY);

    //使用3*3的内核降噪
    Mat edge;
    blur(image1,edge,Size(3,3));
    threshold(edge,image1,150,255,THRESH_BINARY);

    SurfFeatureDetector surfDetector(1000);  //hessianThreshold
    vector<KeyPoint> kp1,kp2;
    surfDetector.detect(image1,kp1);
    surfDetector.detect(image2,kp2);


    drawKeypoints(image1,kp1,image1,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawKeypoints(image2,kp2,image2,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("KeyPoints of map",image1);
    imshow("Keypoints of submap",image2);

    SurfFeatureDetector SurfDescriptor;
    Mat imgd1,imgd2;
    SurfDescriptor.compute(image1,kp1,imgd1);
    SurfDescriptor.compute(image2,kp2,imgd2);

    FlannBasedMatcher matcher;
    vector<DMatch> matPoints;
    matcher.match(imgd1,imgd2,matPoints,Mat());

    //提起强特征
    double min=1,max=0;
    for(unsigned int i=0;i<matPoints.size();i++)
    {
        //匹配最大最值获取
        min = min > matPoints[i].distance ? matPoints[i].distance : min;
        max = max < matPoints[i].distance ? matPoints[i].distance : max;
    }
    vector<DMatch> gmchPoints;

    qDebug()<<"min="<<min<<"max="<<max;
//    sort(matPoints.begin(),matPoints.end()); //从小到大排序
//    for(unsigned int i=0;i<kp2.size()/2;i++)
//    {
//        gmchPoints.push_back(matPoints[i]);
//    }

    for(unsigned int i=0;i<matPoints.size();i++)
    {
        if(matPoints[i].distance<=min)
            gmchPoints.push_back(matPoints[i]);
    }


    Mat imgOutput;
    drawMatches(map,kp1,subMap,kp2,gmchPoints,imgOutput,Scalar::all(-1),Scalar::all(-1),
                vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    imshow("match points",imgOutput);
}


}
