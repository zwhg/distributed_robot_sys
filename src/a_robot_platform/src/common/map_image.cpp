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

void MapImage::GetBinaryImage(const cv::Mat& imgIn,cv::Mat& imgOut)
{
    Mat dst,edge,gray;
    dst.create( imgIn.size(), imgIn.type() );
    // 将原图像转换为灰度图像
    cvtColor( imgIn, gray, CV_BGR2GRAY );
    // 先用使用 3x3内核来降噪
    blur( gray, edge, Size(3,3) );
//    blur( edge, gray, Size(3,3) );
//    blur( gray, edge, Size(3,3) );
    threshold(edge, imgOut, 130, 255, THRESH_BINARY);
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

    GetBinaryImage(map,image1);
    GetBinaryImage(subMap,image2);

    SurfFeatureDetector surfDetector(80);  //hessianThreshold

    vector<KeyPoint> kp1,kp2;
    surfDetector.detect(image1,kp1);
    surfDetector.detect(image2,kp2);
    Mat img1,img2;
    drawKeypoints(image1,kp1,img1,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawKeypoints(image2,kp2,img2,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("KeyPoints of map",img1);
    imshow("Keypoints of submap",img2);

    SurfFeatureDetector SurfDescriptor;
    Mat imgd1,imgd2;
    SurfDescriptor.compute(image1,kp1,imgd1);
    SurfDescriptor.compute(image2,kp2,imgd2);

    FlannBasedMatcher matcher;
    vector<DMatch> matPoints;
    matcher.match(imgd2,imgd1,matPoints,Mat());

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
    sort(matPoints.begin(),matPoints.end()); //从小到大排序
    for(unsigned int i=0;i<kp2.size()/10;i++)
    {
        gmchPoints.push_back(matPoints[i]);
    }

    Mat imgOutput;
    drawMatches(image2,kp2,image1,kp1,gmchPoints,imgOutput,Scalar::all(-1),Scalar::all(-1),
                vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    imshow("match points",imgOutput);
}

void MapImage::OrbFeaturematch(const cv::Mat& map,const cv::Mat &subMap)
{
    if(map.empty() ||subMap.empty())
    {
       qDebug()<< "please open two map image!";
       return;
    }

    Mat img1 =map.clone();
    Mat img2 =subMap.clone();
    ORB orb;
    vector<cv::KeyPoint> kp1, kp2;
    Mat desp1, desp2;
    orb( img1, cv::Mat(), kp1, desp1 );
    orb( img2, cv::Mat(), kp2, desp2 );
    qDebug()<<"map.keypoint="<<kp1.size()<<"submap.keypoint="<<kp2.size();

    Mat image1,image2;
    drawKeypoints(img1,kp1,image1,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawKeypoints(img2,kp2,image2,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("KeyPoints of map",image1);
    imshow("Keypoints of submap",image2);


    Ptr<DescriptorMatcher>  matcher = DescriptorMatcher::create( "BruteForce-Hamming");

    double knn_match_ratio=0.95;
    vector< vector<DMatch> > matches_knn;
    matcher->knnMatch( desp2, desp1, matches_knn, 2 );
    vector<DMatch > matches;
    for ( size_t i=0; i<matches_knn.size(); i++ )
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance )
            matches.push_back( matches_knn[i][0] );
    }

//    vector<Point2f> points1,  points2;
//    for ( auto m:matches )
//    {
//        points1.push_back( kp1[m.queryIdx].pt );
//        points2.push_back( kp2[m.trainIdx].pt );
//    }
    Mat imgOutput;
    drawMatches(img2,kp2,img1,kp1,matches,imgOutput,Scalar::all(-1),Scalar::all(-1),
                vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    imshow("match points",imgOutput);
}

}
