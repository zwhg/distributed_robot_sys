#include <ros/ros.h>
#include "tcpsocketserver.h"
#include "uartodompthread.h"
#include "uartodom.h"
#include <QCoreApplication>
#include <qdebug.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  QCoreApplication a(argc,argv);
  ros::init(argc, argv, "main_uart_odom");
  ROS_INFO("package_name:a_robot_platform  node_name:main_uart_odom");
  qDebug()<<"qDebug ok!";

  zw::TcpSocketServer m_tcpServer;
  zw::UartOdom m_uartOdom;

  int32_t ret =-1;
  const char *addrPort;
  if(argc==1)
    addrPort="/dev/ttyUSB0";
  else
  addrPort =argv[1];

  ret = m_uartOdom.OpenSerial(addrPort, B115200);
  //ret =m_uartOdom.SetNoDelay();
  if(ret <0)
  {
    return -1;
  }
  m_uartOdom.StartScan();

  zw::UartOdomPthread m_OdomPthread;
  a.exec();
  m_uartOdom.CloseSerial();

  ros::shutdown();
  return 0;
}


