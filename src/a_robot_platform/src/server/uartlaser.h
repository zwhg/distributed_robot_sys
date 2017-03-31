#ifndef UARTLASER_H
#define UARTLASER_H

#include "uartdriver.h"

namespace zw
{

#define PACKSIZE 3611
#define PACKLEN (PACKSIZE/5-2)
#define READMAX 1024

union floatTobyte
{
  float  ff;
  uint8_t  fb[4];
};
struct basedata_t{

  int flag;
  int start;
  int end;
  int curr;
  unsigned char data[PACKSIZE];
    basedata_t *next;
};

#pragma pack(1)
struct rplidar_response_measurement_node_t{

  uint8_t    sync_quality;
  uint16_t   angle_q6_checkbit; //角度
  uint16_t   distance_q2;       //距离
};

class UartLaser : public UartDriver
{


public:
    double speed;
    double angle[PACKLEN];
    double distance[PACKLEN];
private:
    basedata_t *pcurr;


  public:
    UartLaser();
    ~UartLaser();


    int32_t GetScanData(double *angle, double *distance, int32_t len, double *speed);
    virtual	int32_t StartScan(void);
    virtual	int32_t StopScan(void);
    virtual	void *DoPthread(void);
    virtual	void Analysis(uint8_t *buf, int nRet);

  private:
    int32_t LsParameter(uint8_t *data, double *angle, double *dist, int32_t len);
    basedata_t *CreateList(void);
    basedata_t *InitList(void);
    void DeletList(void);
};

}
#endif // UARTLASER_H
