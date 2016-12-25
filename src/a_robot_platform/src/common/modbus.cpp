#include "modbus.h"
#include <qdebug.h>



namespace zw {

Modbus modbus;

static pthread_mutex_t g_tMutex  = PTHREAD_MUTEX_INITIALIZER;

static volatile zw::ElementTable dataTable[]=
{
    {CONTROL,0},  //v, w ,x ,y ,theta
    {CONTROL+1,0},
    {CONTROL+2,0},
    {CONTROL+3,0},
    {CONTROL+4,0},

    {MSG_CONTROL,0},
    {MSG_CONTROL+1,0},
    {MSG_CONTROL+2,0},
    {MSG_CONTROL+3,0},
    {MSG_CONTROL+4,0},

    {MSG_IMU,0},
    {MSG_IMU+1,0},
    {MSG_IMU+2,0},
    {MSG_IMU+3,0},
    {MSG_IMU+4,0},
    {MSG_IMU+5,0},

    {MSG_Ultrasonic,0},
    {MSG_Ultrasonic+1,0},
    {MSG_Ultrasonic+2,0},
    {MSG_Ultrasonic+3,0},
    {MSG_Ultrasonic+4,0},
    {MSG_Ultrasonic+5,0},
    {MSG_Ultrasonic+6,0},
    {MSG_Ultrasonic+7,0},
};

uint16_t Modbus::def_cnt=0;

Modbus::Modbus()
{
    int16_t a=1 ;
    endian=(((byte *)&a)[1]!=1);
    dataTableLen =  (sizeof(dataTable))/(sizeof(uint16_t)+sizeof(uint32_t));
    InitCRC16_1201_table();
    def_cnt++;
    if(def_cnt>1)
      qDebug()<<"error,Modbus can only construct once,it has already declare a gobal variable modbus";
}

Modbus::~Modbus()
{

}

bool Modbus::SetAddressValue(const ParaGetSet &para)
{
    uint16_t i,j;
    bool hit=false;

    pthread_mutex_lock(&g_tMutex);
    for(i=0,j=0; (i<dataTableLen)&&(j<para.len); i++)
    {
        if (dataTable[i].addr == para.addr+j)
        {
            dataTable[i].val=para.data[j++];
            hit=true;
        }
        else
        {
            if(hit)
            {
                if(j!=para.len)
                    hit=!hit;
                break;
            }
        }
    }
    pthread_mutex_unlock(&g_tMutex);

    return hit;
}

bool Modbus::GetAddressValue(ParaGetSet &para)
{
    uint16_t i,j;
    bool hit=false;

    pthread_mutex_lock(&g_tMutex);
    for(i=0,j=0; (i<dataTableLen)&&(j<para.len); i++)
    {
        if (dataTable[i].addr == para.addr+j)
        {
            para.data[j++]=dataTable[i].val;
            hit=true;
        }
        else
        {
            if(hit)
            {
                if(j!=para.len)
                    hit =!hit;
                break;
            }
        }
    }
    pthread_mutex_unlock(&g_tMutex);

    return hit;
}

uint16_t Modbus::CalculateCRC16(const byte  buf[], uint16_t size)
{
    uint16_t myCRC16 = 0x0000;
    uint32_t i,j;

    for (i=0; i<size; i++)
    {
        myCRC16 ^= (uint16_t) (buf[i]<<8);
        for (j = 0; j < 8; j++)
        {
            if ((myCRC16 & 0x8000)==0x8000)
                myCRC16 = (uint16_t)((myCRC16 << 1) ^ gx);
            else
                myCRC16 <<= 1;
        }
    }
    return myCRC16;
}

void Modbus::InitCRC16_1201_table(void)
{
    uint16_t i = 0;
    uint8_t buf[1];
    for (i = 0; i < 256; i++)
    {
        buf[0] = i;
        CRC16_1201_table[i] = CalculateCRC16(buf, 1);
    }
}

}


