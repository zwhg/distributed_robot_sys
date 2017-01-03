#include "modbus.h"



namespace zw {

static bool endian=true;
static uint16_t CRC16_1201_table[256];
static const uint16_t gx = 0x1021;
static uint8_t m_cnt=0;

Modbus::Modbus()
{
     if(m_cnt==0)
     {
         int16_t a=1 ;
         endian=(((byte *)&a)[1]!=1);
         InitCRC16_1201_table();
         m_cnt++;
     }
}

Modbus::~Modbus()
{

}

uint16_t Modbus::PackParas(const ParaGetSet& info ,byte outmsg[])
{
    uint16_t i=0;
    outmsg[i++]=HEAD;
    outmsg[i++]=info.fuc;
    Hex2Int16 h2int16;
    if(endian)
    {
        h2int16.i=info.len;
        outmsg[i++]=h2int16.b[0];
        outmsg[i++]=h2int16.b[1];
        h2int16.i=info.addr;
        outmsg[i++]=h2int16.b[0];
        outmsg[i++]=h2int16.b[1];
        if(info.fuc==W_REGISTER){
            for(uint32_t j=0; j<info.len; j++){
                outmsg[i++]=*((byte*)(info.data+j)+0);
                outmsg[i++]=*((byte*)(info.data+j)+1);
                outmsg[i++]=*((byte*)(info.data+j)+2);
                outmsg[i++]=*((byte*)(info.data+j)+3);
            }
        }
    }
    else
    {
        h2int16.i=info.len;
        outmsg[i++]=h2int16.b[1];
        outmsg[i++]=h2int16.b[0];
        h2int16.i=info.addr;
        outmsg[i++]=h2int16.b[1];
        outmsg[i++]=h2int16.b[0];
        if(info.fuc==W_REGISTER){
            for(uint32_t j=0; j<info.len; j++){
                outmsg[i++]=*((byte*)(info.data+j)+3);
                outmsg[i++]=*((byte*)(info.data+j)+2);
                outmsg[i++]=*((byte*)(info.data+j)+1);
                outmsg[i++]=*((byte*)(info.data+j)+0);
            }
        }
    }
   uint16_t crc=0x00;
   if(info.fuc==R_REGISTER)
        crc=CalculateCRC16ByTable(outmsg, CRC_OFFSET, CRC_OFFSET+2);
   else if(info.fuc ==W_REGISTER)
        crc=CalculateCRC16ByTable(outmsg, CRC_OFFSET, CRC_OFFSET+2+info.len*4);

   outmsg[i++] = (byte)(crc & 0xff);
   outmsg[i++] = (byte)(crc >> 8);
   outmsg[i++] =TAIL;
   return i;
}

bool Modbus::UnPackparas(const byte* inMsg, int32_t& startIndex,int32_t endIndex,ParaGetSet & packInfo)
{
    static ParaGetSet Info;
    static uint8_t status=0;
    static uint16_t dat_len=0;

    if(endIndex >= SOCKETBUFMAX )
    {
        qDebug()<<"Recevie too much data !";
        startIndex=endIndex;
        return false;
    }
    
    while (startIndex< endIndex-FIXEDLENGTH)
    {
        switch (status) {
        case 0:{ //find frame head
            if(inMsg[startIndex++]==HEAD)
                status++;            //go to next status
            break;
        }
        case 1:{ //get function code ,and check the function code
                Info.fuc = inMsg[startIndex++];
                if( (Info.fuc == R_REGISTER) ||(Info.fuc == W_REGISTER))
                    status++;
                else
                    status =0;  //return inital status
            break;
        }
        case 2:{ //get length ,and check length
                Info.len= ((uint16_t)((((uint16_t)inMsg[startIndex+1]&0x00ff)<<8 )|
                                      ((uint16_t)inMsg[startIndex]&0x00ff) ));
                if(Info.fuc == R_REGISTER)
                    dat_len=0;
                else if(Info.fuc == W_REGISTER)
                    dat_len=Info.len*4;
                else
                    status=0;

                if(Info.len > EFLMAX_BYTE){
                    status =0;
                }else{
                    status++;  
                }
            break;
        }
        case 3:{ //check tail
            int32_t tailIndex=(int32_t)(startIndex+2+2+dat_len+2);
            if(tailIndex < endIndex){
                if(inMsg[tailIndex] ==TAIL)
                    status++;
                else
                    status=0;
            }else
                return false;
            break;
        }
        case 4:{ //check crc
            int32_t crcIndex= (int32_t)(startIndex+2+2+dat_len);
            uint16_t crc16 = (uint16_t) ((((uint16_t)inMsg[crcIndex+1]&0x00ff)<<8 )|
                                         ((uint16_t)inMsg[crcIndex]&0x00ff));
            if( CalculateCRC16ByTable(inMsg, startIndex+2, crcIndex)==crc16 )
                status++;
            else
                status=0;
            break;
        }
        case 5:{ //get address and data
            uint32_t I=startIndex+2;
            Info.addr =  (int16_t) ((((uint16_t)inMsg[I+1]&0x00ff)<<8 )|
                                      ((uint16_t)inMsg[I]&0x00ff));
             I +=2;
            if(Info.fuc ==R_REGISTER){
                ;
            }else if(Info.fuc ==W_REGISTER){
                if(Info.len !=0){
                    Info.data =new int32_t[Info.len];
                    for(uint16_t i=0; i<Info.len;i++){
                        Info.data[i]=
                            (int32_t)((((uint32_t)inMsg[I+3]&0x000000ff)<<24) |
                            (((uint32_t)inMsg[I+2]&0x000000ff)<<16) |
                            (((uint32_t)inMsg[I+1]&0x000000ff)<<8)  |
                            ((uint32_t)inMsg[I]&0x000000ff) ) ;
                        I +=4;
                    }
                }
            }else{
                status=0;
                return false;
            }
            packInfo=Info;
            startIndex = I+2+1 ;
            status =0;
            return true;
        }
        default:
            break;
        }
    }
    return false ;
}


void Modbus::SendParas(const ParaGetSet & packInfo ,void write(const char* msg,int64_t size))
{
    int32_t size=FIXEDLENGTH +packInfo.len*4;
    byte* msg =new byte[size];
    if(size!=PackParas(packInfo,msg))
        qDebug () <<"Error in pack size!";
    else
        //this->m_readWriteSocket->write((char*)msg ,size);
        write((char*)msg ,(int64_t)size);
    delete msg;
}

uint16_t Modbus::CalculateCRC16ByTable(const byte buf[], int32_t start,int32_t end)
{
    uint16_t myCRC16 = 0x0000;
    uint8_t c = 0;
    int32_t i;

    for (i = start; i < end; i++)
    {
        c = (uint8_t)(myCRC16 >> 8);
        myCRC16 <<= 8;
        myCRC16 ^= CRC16_1201_table[buf[i] ^ c];
    }
    return myCRC16;
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


