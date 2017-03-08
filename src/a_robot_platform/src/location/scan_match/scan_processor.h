#ifndef SCANPROCESSOR_H
#define SCANPROCESSOR_H

#include "sensor_msgs/LaserScan.h"
#include "data_point_container.h"
#include "../../common/map_process.h"
#include "../amcl/map/map.h"
#include  "../amcl/pf/pf_vector.h"

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


class ScanProcessor{

public:
 ScanProcessor();
 ~ScanProcessor();


 Eigen::Vector3f PoseUpdate(const sensor_msgs::LaserScanConstPtr& scan,
                            const Eigen::Vector3f& AmclPoseHintWorld);
 void SetLaserPose(const pf_vector_t& p);

 void GetMultiMap(const map_t* gridMap);

private:

 void LaserScanToDataContainer(const sensor_msgs::LaserScanConstPtr& scan,
                               float scaleToMap);

 Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld,
                           const DataContainer& dataPoints,
                           const map_grid_t* multMap,
                           Eigen::Matrix3f& covMatrix,
                           int maxIterations);

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


public:
 pf_vector_t laser_pose;


private:
 float paramMinDistanceDiffForPoseUpdate;
 float paramMinAngleDiffForPoseUpdate;
 int numDepth;
 map_grid_t *multMap;
 DataContainer dataContainer;


protected:
  Eigen::Vector3f dTr;
  Eigen::Matrix3f H;
  Eigen::Matrix3f lastScanMatchCov;

};


}



#endif

