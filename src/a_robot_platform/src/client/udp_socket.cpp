#include"udp_socket.h"
#include<qdebug.h>
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
                qDebug()<<BtF.ff;
             }
    }
}
