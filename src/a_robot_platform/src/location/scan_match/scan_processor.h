#ifndef SCANPROCESSOR_H
#define SCANPROCESSOR_H

#include "sensor_msgs/LaserScan.h"
#include "data_point_container.h"
#include "../../common/map_process.h"
#include "../amcl/map/map.h"
#include  "../amcl/pf/pf_vector.h"
#include "sensor_msgs/PointCloud.h"

namespace zw {


typedef struct
{
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;

  // Map scale (m/cell)
  double scale;

  // Map dimensions (number of cells)
  int size_x, size_y;

  float *cell_pbb;

} map_grid_t;

typedef struct
{
    Eigen::Vector3f pose;
    float grade;
}poseSet_t;





bool cmp_grade(const poseSet_t& s1,const poseSet_t& s2);

class ScanProcessor{

public:
 ScanProcessor();
 ~ScanProcessor();


 bool PoseUpdate(const sensor_msgs::LaserScanConstPtr& scan,
                            const map_t* map,
                            Eigen::Vector3f& scanPose);
 void SetLaserPose(const pf_vector_t& p);

 void GetMultiMap(const map_t* gridMap);

private:

 void LaserScanToDataContainer(const sensor_msgs::LaserScanConstPtr& scan,
                               float scaleToMap);

 Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld,
                           const DataContainer& dataPoints,
                           const map_grid_t* multMap,
                           Eigen::Matrix3f& covMatrix,
                           int maxIterations,
                           bool & flag );

 bool estimateTransformationLogLh(Eigen::Vector3f& estimate,
                                  const DataContainer& dataPoints,
                                  const map_grid_t* multMap);

 void getCompleteHessianDerivs(const Eigen::Vector3f& pose,
                               const DataContainer& dataPoints,
                               const map_grid_t* multMap,
                               Eigen::Matrix3f& h,
                               Eigen::Vector3f& dTr);

 Eigen::Vector3f interpMapValueWithDerivatives(const map_grid_t* multMap,
                                               const Eigen::Vector2f& coords);

 Eigen::Affine2f getTransformForState(const Eigen::Vector3f& transVector);

 void uniformPoseGenerator(const Eigen::Vector3f& center,
                           const map_t* map,
                           float dwindows,
                           float awindows,
                           int num);

 float getPoseSetGrade(const DataContainer& dataPoints,
                      const map_t* map,
                      const poseSet_t& set,
                      int skip);

 void writePoseToTxt(const char *path,
                     const Eigen::Vector3f& p1,
                     const Eigen::Vector3f& p2,
                     int& i);


 Eigen::Vector3f getBestSet(void);

 void GetSubMap(float factor,const Eigen::Vector3f& finalPose);


public:
 pf_vector_t laser_pose;

 sensor_msgs::PointCloud ptcloud;
 nav_msgs::OccupancyGrid subMap;
 bool getSubmap;



public:
 float poseDiff;
 float angleDiff;
 int numDepth;
 bool publishScan;
 bool writePose;
 int maxIterations;

 int submapFilter;
 int submapExpandCnt;


private:

 map_grid_t *multMap;
 DataContainer dataContainer;

 std::vector<poseSet_t> poseSets;


protected:
  Eigen::Vector3f dTr;
  Eigen::Matrix3f H;
  Eigen::Matrix3f lastScanMatchCov;

};



}


#endif

