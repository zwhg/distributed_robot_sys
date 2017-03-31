#ifndef UARTDRIVER_H
#define UARTDRIVER_H

#include <termios.h>

#include "../common/modbus.h"

namespace zw {



class UartDriver
{
protected:
  int32_t fd;
  pthread_t id;
  volatile int8_t createPthread;
  pthread_mutex_t tMutex;
  pthread_cond_t  tConVar;

public:
  UartDriver();
  ~UartDriver();
  int32_t OpenSerial(const char * addr , uint32_t baudrate);
  int32_t SetNoDelay(void);
  static void *UartCreatePthread(void *tmp);
  void CloseSerial(void);

  virtual	int32_t StartScan(void);
  virtual	int32_t StopScan(void);
  virtual	void *DoPthread(void);
  virtual	void Analysis(uint8_t *buf, int nRet);
};

}



#endif // UARTDRIVER_H
