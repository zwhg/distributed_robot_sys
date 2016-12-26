#include "paras.h"
#include <qdebug.h>



namespace zw {

static uint16_t dataTableLen=0;
static pthread_mutex_t g_tMutex  = PTHREAD_MUTEX_INITIALIZER;
static uint8_t p_cnt=0;

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


Paras::Paras()
{
    if(p_cnt==0){
    dataTableLen =  (sizeof(dataTable))/(sizeof(uint32_t)+sizeof(uint32_t));
    p_cnt++;
    }
}

Paras::~Paras()
{

}

bool Paras::SetAddressValue(const ParaGetSet &para)
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

bool Paras::GetAddressValue(ParaGetSet &para)
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
}


