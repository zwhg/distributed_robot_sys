#include <qdebug.h>
#include "uartodom.h"
#include <unistd.h>


namespace zw {

int32_t UartOdom::StartScan(void)
{
  static uint8_t scanflags = 0;
  createPthread = 1;
  if (scanflags == 0)
  {
    scanflags = 1;
    pthread_create(&id, NULL, UartCreatePthread, (void *)this);
  }
}

int32_t UartOdom::StopScan(void)
{

}

void *UartOdom::DoPthread(void)
{
  int16_t i=0;
  while(createPthread)
  {
    SendVelControl();
    SendCmd2Hard();

    int32_t nRet=0;
    uint8_t data_buf[UARTBUFMAX];
    nRet= read(fd, data_buf, UARTBUFMAX);
    if( nRet >0 )
    {
      Analysis(data_buf, nRet);
    }
//    Paras m_para;
//    int32_t dat[6]={i*10,-i*10,i*5,i+2,-i,i++};
//    ParaGetSet packInfo={W_REGISTER,6, MSG_IMU,dat};
//    m_para.SetAddressValue(packInfo);
//    Float2Int32 fi,fy;
//    fi.f=3.2+i;
//    fy.f=-2.6+i/65535.0;
//    int32_t tmp[2]={fi.i,fy.i};
//    ParaGetSet tmpInfo={W_REGISTER,2, MSG_CONTROL,tmp};
//    m_para.SetAddressValue(tmpInfo);
//      usleep(40000);
    usleep(10000);
  }
}

void UartOdom::Analysis(uint8_t *arry, int nRet)
{

    buf.append((char *)arry,nRet);

    int32_t endIndex = buf.count();
    if(endIndex >=FIXEDLENGTH){
        int32_t startIndex=0;
        ParaGetSet packInfo={NULL1,0,NULL2,nullptr};
        Paras m_para;

        while(m_modbus.UnPackparas((const byte*)buf.data(),startIndex ,endIndex, packInfo))
        {
            if(packInfo.fuc == R_REGISTER){
                packInfo.data=new int32_t[packInfo.len];
                m_para.GetAddressValue(packInfo);
                packInfo.fuc=W_REGISTER;
                int32_t size=FIXEDLENGTH +packInfo.len*4;
                byte* msg =new byte[size];
                if(size!=m_modbus.PackParas(packInfo,msg))
                    qDebug () <<"Error in pack size!";
                else
                    write(fd,(char*)msg ,size);
                delete msg;
                delete packInfo.data;
            } else if(packInfo.fuc == W_REGISTER){
                m_para.SetAddressValue(packInfo);
                int32_t dat[8];
#if 0
                if(packInfo.addr==MSG_Ultrasonic){
                    zw::ParaGetSet  pack = {zw::R_REGISTER,8,zw::MSG_Ultrasonic,dat};
                    m_para.GetAddressValue(pack);
                    zw::Float2Int32 ff[8];
                   for(int i=0;i<8;i++)
                   {
                      ff[i].i = dat[i];
                   }
                   qDebug()<<ff[0].f<<ff[1].f<<ff[2].f<<ff[3].f<<ff[4].f<<ff[5].f<<ff[6].f<<ff[7].f;
                }
#endif
#if 0
                if (packInfo.addr==MSG_IMU)
                {
                	zw::ParaGetSet pack ={zw::R_REGISTER,6,zw::MSG_IMU,dat};
                	m_para.GetAddressValue(pack);
                	qDebug()<<dat[0]<<dat[1]<<dat[2]<<dat[3]<<dat[4]<<dat[5];
                }  
#endif
#if 0              
                if (packInfo.addr==CONTROL)

                }
#endif
#if 0
                if (packInfo.addr==MSG_CONTROL)

                {
                	zw::ParaGetSet pack ={zw::R_REGISTER,2,zw::CONTROL,dat};
                	m_para.GetAddressValue(pack);
                	zw::Float2Int32 ff[2];
                    for(int i=0;i<2;i++)
                    {
                      ff[i].i = dat[i];
                    }
                	qDebug()<<ff[0].f<<ff[1].f;
                }
#endif
#if 0
                if (packInfo.addr==MSG_CONTROL)
                {
                	zw::ParaGetSet pack ={zw::R_REGISTER,2,zw::CONTROL,dat};
                	m_para.GetAddressValue(pack);
                	zw::Float2Int32 ff[2];
               	for(int i=0;i<2;i++)
        		{
        		  ff[i].i = dat[i];
        		}
                	qDebug()<<ff[0].f<<ff[1].f;
                }
#endif

                if(packInfo.data!=nullptr)
                {
                   delete packInfo.data;
                   packInfo.data=nullptr;
                }
            }
        }
        if(startIndex!=0){
            buf.remove(0,startIndex);
            startIndex=0;
        }
    }
}

void UartOdom::SendVelControl(void)
{
    Paras m_para;
    int32_t car_msg[2];
    ParaGetSet car_para={R_REGISTER,2,CONTROL,car_msg};
    m_para.GetAddressValue(car_para);
    Modbus m_modbus;
    car_para.fuc=W_REGISTER;
    int32_t size=FIXEDLENGTH +car_para.len*4;
    byte* msg =new byte[size];
    if(size!=m_modbus.PackParas(car_para,msg))
        qDebug () <<"Error in pack size!";
    else
        write(fd,(char*)msg ,size);
    delete msg;
}

void UartOdom::SendCmd2Hard(void)
{
    Paras m_para;
    int32_t cmd[1];
    bool res=false;
    ParaGetSet cmd_para={R_REGISTER,1,BTN_SWITCH,cmd};
    m_para.GetAddressValue(cmd_para);
    res=((cmd[0]&KEY_INIT_IMU) ==KEY_INIT_IMU)?true:false;
  //  qDebug()<<cmd[0]<<res;
    if(res)
    {
        Modbus m_modbus;
        cmd_para.fuc=W_REGISTER;
        int32_t size=FIXEDLENGTH + cmd_para.len*4;
        byte* msg =new byte[size];
        if(size!=m_modbus.PackParas(cmd_para,msg))
            qDebug () <<"Error in pack size!";
        else{
            write(fd,(char*)msg ,size);
            qDebug()<<"Reset IMU ok!";
        }
        delete msg;
        m_para.ResetKeyRegister();
    }
}

}
