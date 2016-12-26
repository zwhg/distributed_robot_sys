#ifndef PARAMODBUS_H
#define PARAMODBUS_H

#include "modbus.h"
#include <qdebug.h>

namespace zw {


  class ParaModbus
  {
      public:
          uint16_t PackParas(const ParaGetSet& info ,byte outMsg[]);
          bool UnPackparas(const byte* inMsg, int32_t& startIndex,int32_t endIndex,ParaGetSet& packInfo);
          void SendParas(const ParaGetSet & packInfo ,void write(const char* msg,int64_t size));
      private:
          uint16_t CalculateCRC16ByTable(const byte buf[], int32_t start, int32_t end);
  };

}



#endif // PARAMODBUS_H
