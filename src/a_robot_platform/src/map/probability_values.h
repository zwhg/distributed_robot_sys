#ifndef PROBABILITY_VALUES_H
#define PROBABILITY_VALUES_H

#include "../common/common.h"

namespace zw {

constexpr char kOccGrid=100;
constexpr char kFreeGrid=0;
constexpr char kUnknownGrid=-1;

constexpr float kOccProbaility=0.51f;
constexpr float kFreeprobaility=0.49f;
constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;

}



#endif // PROBABILITY_VALUES_H
