#ifndef MODBUS_H
#define MODBUS_H

#include "common.h"

namespace zw {

#pragma pack(1)
typedef struct
{
    volatile uint16_t addr ;
    volatile int32_t val ;
}ElementTable;

#pragma pack()
#pragma pack(1)
typedef struct
{
    byte fuc;
    uint16_t len;   //int32_t unit
    uint16_t addr ;
    int32_t* data ;
}ParaGetSet;

typedef enum
{
    HEAD=0x5a,
    TAIL=0xa5
}FrameHeadAndTail;

#pragma pack()
#pragma pack(1)
typedef struct
{
    static const byte head=HEAD;
    ParaGetSet pack;
    uint16_t crc;
    static const byte tail=TAIL;
}ModbusMsg;

typedef enum
{
     R_HOLDING_REGISTER =0x03,
     W_MULTI_REGISTER =0x10,
     NULL1
}FunctionCode;

typedef enum
{
    CONTROL =0x0000,   // v , w ,x ,y ,theta   //float
    MSG_CONTROL =0x0020, //show msg  v,w,x,y,theta   //float
    MSG_IMU = 0x0040,  //show msg  acc_x,y,z  gyr_x_y_z   //int16_t
    MSG_Ultrasonic = 0x0060,  //超声  int
}ParaAddress;


#define FIXEDLENGTH     9
#define CRC_OFFSET      4
#define SOCKETBUFMAX 	10000
#define EFLMAX_BYTE		256	// the maximum effective frame length


class Modbus
{
public:
    bool endian;
    uint16_t dataTableLen;
    uint16_t CRC16_1201_table[256];
    static const uint16_t gx = 0x1021;
    static uint16_t def_cnt;

public:
    Modbus();
    ~Modbus();
    bool SetAddressValue(const ParaGetSet &para);
    bool GetAddressValue(ParaGetSet &para);
private:
    uint16_t CalculateCRC16(const byte  buf[], uint16_t size);
    void InitCRC16_1201_table(void);
};

  extern  Modbus modbus;

}

#endif // MODBUS_H
