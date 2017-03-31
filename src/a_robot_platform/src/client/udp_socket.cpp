#include"udp_socket.h"
#include<qdebug.h>

float  laser_dis[720]={0};

namespace zw
{

    UdpSocket::UdpSocket(QObject *parent):
        QObject(parent)
    {
       //qDebug()<<"UdpSocket is running!";
        m_udpClient = new QUdpSocket(this);
        m_udpClient->bind(45454,QUdpSocket::ShareAddress);
        QObject::connect(m_udpClient,SIGNAL(readyRead()),this,SLOT(processPendingDatagram()));
    }

    UdpSocket::~UdpSocket()
    {
        delete m_udpClient;
    }

    void UdpSocket::processPendingDatagram()
    {
           int data_num=0;
           while(m_udpClient->hasPendingDatagrams())
            {
                QByteArray  datagram;
                floatTobyte  BtF;
                datagram.resize(m_udpClient->pendingDatagramSize());
                //receive message
                m_udpClient->readDatagram(datagram.data(),datagram.size());
                for(int i=0;i<4;i++)
                {
                    BtF.fb[i] = datagram.at(i);
                }
                if(data_num<720)
                {
                    laser_dis[data_num++] = BtF.ff;
                   // qDebug()<<"laser_disXXXX"<<laser_dis[data_num];
                }
                else
                    data_num=0;
             }
    }
}
