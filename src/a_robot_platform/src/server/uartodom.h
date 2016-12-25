#ifndef UARTODOM_H
#define UARTODOM_H

#include "uartdriver.h"
#include "../common/common.h"
#include "../common/paramodbus.h"

namespace zw {

#define UARTBUFMAX 	5000



class UartOdom: public UartDriver
{

public:
  ParaModbus m_paraModbus;

public:
  virtual	int32_t StartScan(void);
  virtual	int32_t StopScan(void);
  virtual	void *DoPthread(void);
  virtual	void Analysis(uint8_t *buf, int nRet);

};


}

#endif // UARTODOM_H
