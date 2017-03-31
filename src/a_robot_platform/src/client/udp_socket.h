#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <QObject>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/qudpsocket.h>

extern float laser_dis[720];

namespace  zw{

    union floatTobyte
   {
      float  ff;
      uint8_t  fb[4];
   };

class UdpSocket : public QObject
    {
       Q_OBJECT
public:
        explicit UdpSocket(QObject *parent=0);
      ~UdpSocket();
private:
      QUdpSocket *m_udpClient ;
public slots:
      void processPendingDatagram();
    };
}
#endif // UDP_SOCKET_H
