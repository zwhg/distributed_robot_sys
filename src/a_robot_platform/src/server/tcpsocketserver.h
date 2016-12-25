#ifndef TCPSOCKETSERVER_H
#define TCPSOCKETSERVER_H

#include <QObject>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QTcpServer>

#include "../common/paramodbus.h"

namespace zw {

class TcpSocketServer : public QObject
{
  Q_OBJECT

private:
    ParaModbus m_paraModbus;
    QTcpServer *m_listenSocket;
    QTcpSocket *m_readWriteSocket;

public:
   TcpSocketServer();
  ~TcpSocketServer();

private slots:
    void on_ProcessConnection();
    void on_ReadyRead(void);
};

}



#endif // TCPSOCKETSERVER_H
