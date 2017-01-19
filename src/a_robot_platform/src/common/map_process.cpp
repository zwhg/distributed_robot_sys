#include "map_process.h"

namespace zw {


void map_filter(const char * dat, char *out,uint32_t w, uint32_t h)
{
  for(uint32_t i=0;i<w*h;i++)
      out[i]=dat[i];

  for(uint32_t x=1;x<w-1;x++)
  {
    for(uint32_t y=1;y<h-1;y++)
    {
      uint32_t ci=GetGridIndexOfMap(w,x,y);
      uint32_t cl=ci-1;
      uint32_t cr=ci+1;
      uint32_t cu=GetGridIndexOfMap(w,x,(y+1));
      uint32_t cul=cu-1;
      uint32_t cur=cu+1;
      uint32_t cd=GetGridIndexOfMap(w,x,(y-1));
      uint32_t cdl=cd-1;
      uint32_t cdr=cd+1;

      if((dat[ci]!=dat[cl])&&(dat[cl]==dat[cr])&&
         (dat[cl]==dat[cu])&&(dat[cl]==dat[cd]))
        out[ci]=dat[cl];
    }
  }

}

}

