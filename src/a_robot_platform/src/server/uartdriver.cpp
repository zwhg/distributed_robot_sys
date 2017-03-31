#include  <stdio.h>      /*标准输入输出定义*/
#include  <stdlib.h>     /*标准函数库定义*/
#include  <unistd.h>     /*Unix 标准函数定义*/
#include  <sys/types.h>
#include  <sys/stat.h>
#include  <fcntl.h>      /*文件控制定义*/
#include  <termios.h>    /*PPSIX 终端控制定义*/
#include  <errno.h>      /*错误号定义*/
#include  <qdebug.h>
#include "uartdriver.h"
//#include <ros/ros.h>

namespace zw {

UartDriver::UartDriver()
{
  pthread_mutex_init( &tMutex,NULL);
  pthread_cond_init( &tConVar,NULL);
  createPthread = 0;
}

UartDriver:: ~UartDriver()
{

}


int32_t UartDriver::OpenSerial(const char * addr , uint32_t baudrate)
{
  struct termios  m_stNew;
  struct termios  m_stOld;

  fd = open(addr, O_RDWR|O_NOCTTY|O_NDELAY);
  if(fd<0)
  {
   // ROS_FATAL("[%s - %d] Open %s  error ! \n", __FILE__, __LINE__, addr);
    qDebug("Open %s error !",addr);
    return -1;
  }
  if( (fcntl(fd, F_SETFL, 0)) < 0 )  //FNDELAY
  {
    //ROS_FATAL("[%s - %d] Fcntl F_SETFL Error ! \n", __FILE__, __LINE__);
    qDebug()<<"Fcntl F_SETFL Error !";
    return -1;
  }
  if(tcgetattr(fd, &m_stOld) != 0)
  {
    //ROS_FATAL("[%s - %d] tcgetattr error ! \n", __FILE__, __LINE__);
    qDebug()<<"tcgetattr error !";
    return -1;
  }

  m_stNew = m_stOld;
  //将终端设置为原始模式，该模式下所有的输入数据以字节为单位被处理
  cfmakeraw(&m_stNew);

  //set speed
  cfsetispeed(&m_stNew, baudrate);
  cfsetospeed(&m_stNew, baudrate);
  //set databits
  m_stNew.c_cflag |= (CLOCAL|CREAD);
  m_stNew.c_cflag &= ~CSIZE;
  m_stNew.c_cflag |= CS8;
  //set parity
  m_stNew.c_cflag &= ~PARENB;
  m_stNew.c_iflag &= ~INPCK;
    //set stopbits
  m_stNew.c_cflag &= ~CSTOPB;
  m_stNew.c_cc[VTIME]=0;	//指定所要读取字符的最小数量
  m_stNew.c_cc[VMIN]=1;	//指定读取第一个字符的等待时间，时间的单位为n*100ms
  //如果设置VTIME=0，则无字符输入时read（）操作无限期的阻塞
  tcflush(fd,TCIFLUSH);	//清空终端未完成的输入/输出请求及数据。
  if( tcsetattr(fd,TCSANOW,&m_stNew) != 0 )
  {
    //ROS_FATAL("[%s - %d] tcsetattr error ! \n", __FILE__, __LINE__);
    qDebug()<< "tcsetattr error !";
    return -1;
  }
  return fd;
}

int32_t UartDriver::SetNoDelay(void)
{
  if( (fcntl(fd, F_SETFL, FNDELAY)) < 0 )  //FNDELAY
  {
   // ROS_FATAL("[%s - %d] FNDELAY Error ! \n", __FILE__, __LINE__);
    qDebug()<< "FNDELAY Error !";
    return -1;
  }
  return fd;
}

void *UartDriver::UartCreatePthread(void *tmp)
{
  UartDriver * t =(UartDriver *)tmp;
  t->DoPthread();
}

void UartDriver::CloseSerial(void)
{
  createPthread = 0;
  usleep(10000);
  //pthread_join(id,NULL);   //
  pthread_detach(id);

  if(fd>=0)
    close(fd);
}

int32_t UartDriver::StartScan(void)
{

}

int32_t UartDriver::StopScan(void)
{

}

void *UartDriver::DoPthread(void)
{

}

void UartDriver::Analysis(uint8_t *buf, int nRet)
{

}

}


