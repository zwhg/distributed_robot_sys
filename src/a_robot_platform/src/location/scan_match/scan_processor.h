#ifndef SCANPROCESSOR_H
#define SCANPROCESSOR_H

#include "sensor_msgs/LaserScan.h"
#include "data_point_container.h"
#include "../../common/map_process.h"
#include "../amcl/map/map.h"
#include  "../amcl/pf/pf_vector.h"

namespace zw {


class ScanProcessor{

public:
 ScanProcessor();
 ~ScanProcessor();
 bool LaserScanToDataContainer(const sensor_msgs::LaserScanConstPtr& scan,
                               DataContainer& dataContainer,
                               float scaleToMap);

 Eigen::Vector3f PoseUpdate(const DataContainer& dataContainer,
                 const map_t* gridMap,
                 const Eigen::Vector3f& AmclPoseHintWorld);
 void SetLaserPose(const pf_vector_t& p);

private:
 bool PoseDifferenceLargerThan(const Eigen::Vector3f& pose1,                              
                               const Eigen::Vector3f& pose2,
                               float distanceDiffThresh,
                               float angleDiffThresh);
 Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld,
                           const map_t* gridMap,
                           const DataContainer& dataContainer,
                           Eigen::Matrix3f& covMatrix,
                           int maxIterations);
 bool estimateTransformationLogLh(Eigen::Vector3f& estimate,
                                  const map_t* gridMap,
                                  const DataContainer& dataPoints);

 void getCompleteHessianDerivs(const map_t* gridMap,
                               const Eigen::Vector3f& pose,
                               const DataContainer& dataPoints,
                               Eigen::Matrix3f& h,
                               Eigen::Vector3f& dTr);
 Eigen::Vector3f interpMapValueWithDerivatives(const map_t* gridMap,
                                               const Eigen::Vector2f& coords);

 Eigen::Affine2f getTransformForState(const Eigen::Vector3f& transVector);


public:
 pf_vector_t laser_pose;
 DataContainer dataContainer;

private:
 float paramMinDistanceDiffForPoseUpdate;
 float paramMinAngleDiffForPoseUpdate;

protected:
  Eigen::Vector3f dTr;
  Eigen::Matrix3f H;
  Eigen::Matrix3f lastScanMatchCov;

};


}



#endif

