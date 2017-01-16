#ifndef PARAS_H
#define PARAS_H

#include "common.h"

namespace zw {

#pragma pack(4)
typedef struct
{
    volatile uint16_t addr ;
    volatile int32_t val ;
}ElementTable;

typedef struct
{
    byte fuc;
    uint16_t len;   //int32_t unit
    uint16_t addr ;
    int32_t* data ;
}ParaGetSet;

typedef enum
{
    CONTROL =0x0000,   // v , w ,x ,y ,theta   //float
    MSG_CONTROL =0x0020, //show msg  v,w,x,y,theta   //float
    MSG_IMU = 0x0040,  //show msg  acc_x,y,z  gyr_x_y_z   //int16_t
    MSG_Ultrasonic = 0x0060,  //超声  int
}ParaAddress;


class Paras
{
public:
    Paras();
    ~Paras();
    bool SetAddressValue(const ParaGetSet &para);
    bool GetAddressValue(ParaGetSet &para);
    static void get_distance(double *dis);
    static void set_distance(double *dis);

};

}

#endif // PARAS_H
