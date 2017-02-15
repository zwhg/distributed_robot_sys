#ifndef MAP_PROCESS_H
#define MAP_PROCESS_H

#include "common.h"
#include <assert.h>
#include "eigen3/Eigen/Eigen"
#include "../location/amcl/map/map.h"

namespace zw {

constexpr char kOccGrid=100;
constexpr char kFreeGrid=0;
constexpr char kUnknownGrid=-1;

constexpr float kOccProbaility=0.51f;
constexpr float kUnknownProbability=0.5f;
constexpr float kFreeprobaility=0.49f;
constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;



// map coords :low-left

// compute linear index for given map coords
inline uint32_t GetGridIndexOfMap(uint32_t w,uint32_t x,uint32_t y)
{
  uint32_t index= w*y+x;
 // assert(index>=0 && index < w*h);
  return  index;
}

/**
 * Returns the map pose for the given world pose.
 */
inline Eigen::Vector3f getMapCoordsPose(const map_t* map, const Eigen::Vector3f& worldPose)
{
    return Eigen::Vector3f(MAP_GXWX(map, worldPose[0]),MAP_GYWY(map, worldPose[1]),
            worldPose[2]);
}

/**
 * Returns the world pose for the given map pose.
 */
inline Eigen::Vector3f getWorldCoordsPose(const map_t* map, const Eigen::Vector3f& mapPose)
{
    return Eigen::Vector3f(MAP_WXGX(map, mapPose[0]),MAP_WYGY(map, mapPose[1]),
            mapPose[2]);
}

/**
 * Compute the map coords for given the cell index
 */





void map_filter(const char * dat, char *out,uint32_t w, uint32_t h);

}



#endif // MAP_PROCESS_H
