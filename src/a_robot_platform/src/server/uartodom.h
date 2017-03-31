#ifndef UARTODOM_H
#define UARTODOM_H

#include "uartdriver.h"
#include "../common/common.h"
#include "../common/modbus.h"

namespace zw {

#define UARTBUFMAX 	5000
class UartOdom: public UartDriver
{

public:
  Modbus m_modbus;
private:
  QByteArray buf;

public:
  virtual	int32_t StartScan(void);
  virtual	int32_t StopScan(void);
  virtual	void *DoPthread(void);
  virtual	void Analysis(uint8_t *buf, int nRet);

private:
  void SendVelControl(void);
  void SendCmd2Hard(void);
};


}

#endif // UARTODOM_H
