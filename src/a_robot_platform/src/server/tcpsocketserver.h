#ifndef TCPSOCKETSERVER_H
#define TCPSOCKETSERVER_H

#include <QObject>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QTcpServer>

#include "../common/modbus.h"

namespace zw {

class TcpSocketServer : public QObject
{
  Q_OBJECT

private:
    Modbus m_modbus;
    QTcpServer *m_listenSocket;
    QTcpSocket *m_readWriteSocket;
    QByteArray buf;
    volatile int cmd_time_out;

public:
   TcpSocketServer();
  ~TcpSocketServer();

private slots:
    void on_ProcessConnection();
    void on_ReadyRead(void);
    void cmdTimeOut();
};

}



#endif // TCPSOCKETSERVER_H
