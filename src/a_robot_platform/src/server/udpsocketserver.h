#ifndef UDPSOCKETSERVER_H
#define UDPSOCKETSERVER_H

#include <QObject>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/qudpsocket.h>

class QUdpSocket;

namespace zw{

    class UdpSocketServer : public QObject
    {
      Q_OBJECT
  public:
      UdpSocketServer();
      ~UdpSocketServer();
  private:
      QUdpSocket *Udp_Sender;

    };
}
#endif // UDPSOCKETSERVER_H
