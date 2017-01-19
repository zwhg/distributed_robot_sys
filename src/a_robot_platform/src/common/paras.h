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
    BTN_SWITCH =0x0080,   //状态开关，键盘控制(bit 0)
}ParaAddress;

#define KEY_VEL_CTR     0x00000001


#define Gyro_Gr	  (2000.0/65536.0*3.1415926/180.0)//(0.0005326/2.0)
#define Acc_Mss   ((4.0*9.81)/65536.0)


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
