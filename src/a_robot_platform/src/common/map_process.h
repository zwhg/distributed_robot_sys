#ifndef MAP_PROCESS_H
#define MAP_PROCESS_H

#include "common.h"
#include <assert.h>

namespace zw {

// compute linear index for given map coords
// map coords :low-left

inline uint32_t GetGridIndexOfMap(uint32_t w,uint32_t x,uint32_t y)
{
  uint32_t index= w*y+x;
 // assert(index>=0 && index < w*h);
  return  index;
}

void map_filter(const char * dat, char *out,uint32_t w, uint32_t h);

}



#endif // MAP_PROCESS_H
