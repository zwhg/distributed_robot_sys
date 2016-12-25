#ifndef PARAMODBUS_H
#define PARAMODBUS_H

#include "modbus.h"

namespace zw {


  class ParaModbus
  {
      public:
          uint16_t PackParas(const ParaGetSet& info ,byte outMsg[]);
          bool UnPackparas(const byte* inMsg, int32_t& startIndex,int32_t endIndex,ParaGetSet& packInfo);

      private:
          uint16_t CalculateCRC16ByTable(const byte buf[], int32_t start, int32_t end);
  };

}



#endif // PARAMODBUS_H
