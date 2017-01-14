#ifndef PROBABILITY_VALUES_H
#define PROBABILITY_VALUES_H

#include "../common/common.h"

namespace zw {

constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;

}



#endif // PROBABILITY_VALUES_H
