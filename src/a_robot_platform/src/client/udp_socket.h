#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <QObject>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/qudpsocket.h>
namespace  zw{
    class UdpSocketClient : public QObject
    {
       Q_OBJECT
 public:
        UdpSocketClient();
      ~UdpSocketClient();
private:
      QUdpSocket *m_udpClient ;
    };
}
#endif // UDP_SOCKET_H
