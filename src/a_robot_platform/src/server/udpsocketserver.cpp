#include "udpsocketserver.h"
#include <QtNetwork>
#include "uartlaser.h"

namespace zw {
      UdpSocketServer::UdpSocketServer()
      {
          qDebug()<<"this is laser constructor";
         double laser_dis[PACKLEN];
         Paras::get_distance(laser_dis);
         Udp_Sender = new QUdpSocket(this);
         QByteArray datagram;
         floatTobyte  FtB;
         for(int i=0;i<720;i++)
         {
              FtB.ff =  laser_dis[i];
              for(int j=0;j<4;j++)
              {
                    datagram.append(FtB.fb[j]);
              }
               Udp_Sender->writeDatagram(datagram.data(),datagram.size(),QHostAddress::Broadcast,45454);
               datagram.clear();
         }
      }
      UdpSocketServer::~UdpSocketServer()
      {
          delete  Udp_Sender;
      }
}

