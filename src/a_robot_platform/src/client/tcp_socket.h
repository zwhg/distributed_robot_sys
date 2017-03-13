#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H


#include <QtNetwork/QTcpSocket>
#include <QObject>
#include "../../common/paras.h"
#include "../../common/modbus.h"

namespace zw {


    typedef enum
    {
       DISCONNECTED=0,        //initial status
       CONNECTING,
       CONNECTED,
       LOSECONNECT
    }ConnectStatus;


   class TcpSocket  : public QObject{

       Q_OBJECT

       public:
           Modbus m_modbus ;
           bool doConnect;
           ConnectStatus m_connectStatus;
           std::string host ;
           uint16_t port ;

       private:
           QTcpSocket *m_tcpClient;
           static const uint32_t CONNECT_TIMEOUT=3;  //s
           bool isConnected;
           QByteArray buf;

       public:
           explicit  TcpSocket(QObject *parent=0);
           ~TcpSocket();
           bool Connect(void);
           void DisConnect(void);      
           bool SendMsg(const byte *bytes,uint16_t size);
           bool SendMsg(ParaGetSet &PackInfo);

       private:
           bool IsConnect(void);
           void DoConnect(std::string host ,uint16_t port);



       public slots:
           void on_Connected();
           void on_Disconnected();
           void on_ReadyRead();

   };
}


#endif // TCP_SOCKET_H
