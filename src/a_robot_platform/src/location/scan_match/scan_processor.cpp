#include "scan_processor.h"
#include <ros/ros.h>


namespace zw{


ScanProcessor::ScanProcessor()
{
    paramMinDistanceDiffForPoseUpdate=0.1;
    paramMinAngleDiffForPoseUpdate=5;
}

ScanProcessor::~ScanProcessor()
{

}


Eigen::Vector3f ScanProcessor::PoseUpdate(const DataContainer& dataContainer,
                               const map_t* gridMap,
                               const Eigen::Vector3f& AmclPoseHintWorld)
{
    Eigen::Vector3f newPoseEstimateWorld;

    newPoseEstimateWorld = matchData(AmclPoseHintWorld,gridMap,dataContainer,lastScanMatchCov,8);

    if(PoseDifferenceLargerThan(newPoseEstimateWorld, AmclPoseHintWorld,
                                paramMinDistanceDiffForPoseUpdate, paramMinAngleDiffForPoseUpdate))
   {
      //...
   }
    return newPoseEstimateWorld;
}

/*
arg1 : t-1时刻机器人在世界坐标系下的位姿
arg2 : 栅格地图
arg3 : 激光数据
arg4 : 当前时刻 hassian  矩阵
arg5 : 最大迭代次数
ret : t时刻机器人在世界坐标系下的位姿
*/
Eigen::Vector3f ScanProcessor::matchData(const Eigen::Vector3f& beginEstimateWorld,
                          const map_t* map,
                          const DataContainer& dataContainer,
                          Eigen::Matrix3f& covMatrix,
                          int maxIterations)
{
    if (dataContainer.getSize() != 0) {
        //计算激光坐标系在全局坐标中的坐标
        // Take account of the laser pose relative to the robot
         pf_vector_t pose={beginEstimateWorld[0],beginEstimateWorld[1],beginEstimateWorld[2]};  //robot in world pose
         pose = pf_vector_add(pose,laser_pose);    //laser in world pose

      //   std::cout<<"lwp: [ "<<pose.v[0]<<" "<<pose.v[1]<<" "<<pose.v[2]<<" ]"<<"\n";

         int mi,mj;
         mi=MAP_GXWX(map, pose.v[0]);
         mj=MAP_GYWY(map, pose.v[1]);
         if(!MAP_VALID(map, mi, mj)){
             ROS_ERROR("scan match origin pose not in scale");
             return beginEstimateWorld;
         }
         Eigen::Vector3f estimate(mi,mj,pose.v[2]);  //laser in map pose

      //   std::cout<<"lmp: [ "<<estimate[0]<<" "<<estimate[1]<<" "<<estimate[2]<<" ]"<<"\n";

         for(int i=0;i<maxIterations+1;i++)
         {
             estimateTransformationLogLh(estimate,map,dataContainer);
         }

        //normalize angle
         float angle = fmod(fmod(estimate[2], 2.0f*M_PI) + 2.0f*M_PI, 2.0f*M_PI);
         if(angle>M_PI)
             angle -= 2.0f*M_PI;
         estimate[2]=angle;

         covMatrix = Eigen::Matrix3f::Zero();
         covMatrix = H;

       //  std::cout<<"elmp: [ "<<estimate[0]<<" "<<estimate[1]<<" "<<estimate[2]<<" ]"<<"\n";
         //laser in world pose
         pose={MAP_WXGX(map,estimate[0]),MAP_WYGY(map,estimate[1]),estimate[2]};
     //    std::cout<<"elwp: [ "<<pose.v[0]<<" "<<pose.v[1]<<" "<<pose.v[2]<<" ]"<<"\n";
    //     std::cout<<"map origin = ["<<map->origin_x<<" "<<map->origin_y<<" ]"<<"\n";

         pose= pf_vector_sub(pose,laser_pose);  //robot in world pose

    //     std::cout<<"rwp: [ "<<pose.v[0]<<" "<<pose.v[1]<<" "<<pose.v[2]<<" ]"<<"\n";

         estimate[0]=pose.v[0];
         estimate[1]=pose.v[1];
         estimate[2]=pose.v[2];
         return estimate;
    }
    return beginEstimateWorld;
}


/*计算 hessian 矩阵 ，并估计t时刻，机器人的位姿*/
bool ScanProcessor::estimateTransformationLogLh(Eigen::Vector3f& estimate,
                                 const map_t* gridMapUtil,
                                 const DataContainer& dataPoints)
{
    getCompleteHessianDerivs(gridMapUtil,estimate, dataPoints, H, dTr);

//    std::cout <<"size = "<<dataPoints.getSize()<<" N = "<<n<<"\n";
//    std::cout << "\nH\n" << H  << "\n";
//    std::cout << "\ndTr\n" << dTr  << "\n";


    if ((H(0, 0) != 0.0f) && (H(1, 1) != 0.0f)) {

      //H += Eigen::Matrix3f::Identity() * 1.0f;
      Eigen::Vector3f searchDir (H.inverse() * dTr);

    //  std::cout << "\nsearchdir\n" << searchDir  << "\n";

      if (searchDir[2] > 0.2f) {
        searchDir[2] = 0.2f;
        ROS_INFO("SearchDir angle change too large\n");;
      } else if (searchDir[2] < -0.2f) {
        searchDir[2] = -0.2f;
        ROS_INFO("SearchDir angle change too large\n");;
      }
      estimate += searchDir;
      return true;
    }
    return false;
}

void ScanProcessor::getCompleteHessianDerivs(const map_t *gridMapUtil,
                                             const Eigen::Vector3f& pose,
                                             const DataContainer& dataPoints,
                                             Eigen::Matrix3f& h,
                                             Eigen::Vector3f& dTr)
{
     int size = dataPoints.getSize();
     Eigen::Affine2f transform(getTransformForState(pose));  //laser pose in map transform

     float sinRot = sin(pose[2]);
     float cosRot = cos(pose[2]);

     h = Eigen::Matrix3f::Zero();
     dTr = Eigen::Vector3f::Zero();

     for (int i = 0; i < size; ++i) {

       const Eigen::Vector2f& currPoint (dataPoints.getVecEntry(i));  //end point in laser pose
       // end point in map pose
       const Eigen::Vector2f endPoint =transform * currPoint;

       Eigen::Vector3f transformedPointData(interpMapValueWithDerivatives(gridMapUtil,endPoint));

       float funVal = 1.0f - transformedPointData[0];

       dTr[0] += transformedPointData[1] * funVal;
       dTr[1] += transformedPointData[2] * funVal;

       float rotDeriv = ((-sinRot * currPoint.x() - cosRot * currPoint.y()) * transformedPointData[1] +
                         (cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2]);

       dTr[2] += rotDeriv * funVal;

       h(0, 0) += transformedPointData[1]*transformedPointData[1];
       h(1, 1) += transformedPointData[2]*transformedPointData[2];
       h(2, 2) += rotDeriv*rotDeriv;

       h(0, 1) += transformedPointData[1] * transformedPointData[2];
       h(0, 2) += transformedPointData[1] * rotDeriv;
       h(1, 2) += transformedPointData[2] * rotDeriv;
     }

     h(1, 0) = h(0, 1);
     h(2, 0) = h(0, 2);
     h(2, 1) = h(1, 2);
}

Eigen::Vector3f ScanProcessor::interpMapValueWithDerivatives(const map_t *gridMapUtil,
                                              const Eigen::Vector2f& coords)
{
  if(!MAP_VALID(gridMapUtil, coords[0],coords[1]))
  {
      return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  }
  //map coords are always positive, floor them by casting to int
   Eigen::Vector2i indMin(coords.cast<int>());

   //get factors for bilinear interpolation
   Eigen::Vector2f factors(coords - indMin.cast<float>());
   int sizeX = gridMapUtil->size_x;
   int index = indMin[1] * sizeX + indMin[0];
   float intensities[4];

   intensities[0] = gridMapUtil->cells[index].probability;
   ++index;
   intensities[1] = gridMapUtil->cells[index].probability;
   index += sizeX-1;
   intensities[2] = gridMapUtil->cells[index].probability;
   ++index;
   intensities[3] = gridMapUtil->cells[index].probability;

   float dx1 = intensities[0] - intensities[1];
   float dx2 = intensities[2] - intensities[3];

   float dy1 = intensities[0] - intensities[2];
   float dy2 = intensities[1] - intensities[3];

   float xFacInv = (1.0f - factors[0]);
   float yFacInv = (1.0f - factors[1]);

   return Eigen::Vector3f(
     ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
     ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),
     -((dx1 * yFacInv) + (dx2 * factors[1])),
     -((dy1 * xFacInv) + (dy2 * factors[0]))
   );
}

Eigen::Affine2f ScanProcessor::getTransformForState(const Eigen::Vector3f& transVector)
{
  return Eigen::Translation2f(transVector[0], transVector[1]) * Eigen::Rotation2Df(transVector[2]);
}

bool ScanProcessor::LaserScanToDataContainer(const sensor_msgs::LaserScanConstPtr& scan,
                                             DataContainer &dataContainer,
                                             float scaleToMap)
{
    size_t size = scan->ranges.size();
    float angle = scan->angle_min;
    dataContainer.clear();
    dataContainer.setOrigo(Eigen::Vector2f::Zero());
    float maxRangeForContainer = scan->range_max - 0.1f;

    for (size_t i = 0; i < size; ++i)
    {
        float dist = scan->ranges[i];
        if ( (dist > scan->range_min) && (dist < maxRangeForContainer))
        {
            dist *= scaleToMap;
            dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
        }
        angle += scan->angle_increment;
    }

  //  std::cout<<"laser size = "<<size<<"\n";

    return true;
}

bool ScanProcessor::PoseDifferenceLargerThan(const Eigen::Vector3f& pose1,
                                             const Eigen::Vector3f& pose2,
                                             float distanceDiffThresh,
                                             float angleDiffThresh)
{
    //check distance
    if ( ( (pose1.head<2>() - pose2.head<2>()).norm() ) > distanceDiffThresh)
        return true;

    float angleDiff = (pose1.z() - pose2.z());

    if (angleDiff > M_PI) {
        angleDiff -= M_PI * 2.0f;
    } else if (angleDiff < -M_PI) {
        angleDiff += M_PI * 2.0f;
    }

    if (abs(angleDiff) > angleDiffThresh)
        return true;
    return false;
}

void ScanProcessor::SetLaserPose(const pf_vector_t& p)
{
    this->laser_pose=p;
}

}


