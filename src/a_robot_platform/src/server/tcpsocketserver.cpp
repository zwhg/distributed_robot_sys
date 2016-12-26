#include <qdebug.h>
#include <QtNetwork/QHostAddress>
#include "tcpsocketserver.h"

namespace zw {

TcpSocketServer::TcpSocketServer()
{
    m_listenSocket =new QTcpServer();
    m_listenSocket->listen(QHostAddress::Any , SOCKET_PORT);
    QObject::connect(this->m_listenSocket,SIGNAL(newConnection()),this,SLOT(on_ProcessConnection()));

}

TcpSocketServer::~TcpSocketServer()
{
    delete m_listenSocket;
}

void TcpSocketServer::on_ProcessConnection(void)
{
    this->m_readWriteSocket =this->m_listenSocket->nextPendingConnection();
    QObject::connect(this->m_readWriteSocket,SIGNAL(readyRead()),this,SLOT(on_ReadyRead()));
    qDebug() << "connected...";
}

void TcpSocketServer::on_ReadyRead(void)
{
    static QByteArray buf;
    QByteArray arry= this->m_readWriteSocket->readAll();
    buf.append(arry);
    int32_t startIndex=0;
    int32_t endIndex = buf.count();
    ParaGetSet packInfo={0,0,0,nullptr};

    while(m_paraModbus.UnPackparas((const byte*)buf.data(),startIndex ,endIndex, packInfo))
    {
        if(packInfo.fuc == R_HOLDING_REGISTER){
            packInfo.data=new int32_t[packInfo.len];
            modbus.GetAddressValue(packInfo);
            packInfo.fuc=W_MULTI_REGISTER;
            int32_t size=FIXEDLENGTH +packInfo.len*4;
            byte* msg =new byte[size];
            if(size!=m_paraModbus.PackParas(packInfo,msg))
                qDebug () <<"Error in pack size!";
            else
                this->m_readWriteSocket->write((char*)msg ,size);
            delete msg;
        }
        else if(packInfo.fuc == W_MULTI_REGISTER){
            modbus.SetAddressValue(packInfo);

//            int32_t dat[2];
//            zw::ParaGetSet  packInfo = {zw::R_HOLDING_REGISTER,2,zw::CONTROL,dat};
//            modbus.GetAddressValue(packInfo);
//            zw::Float2Int32 ff1,ff2;
//            ff1.i=dat[0];
//            ff2.i=dat[1];
//            qDebug () <<ff1.f<<ff2.f;
        }
        delete packInfo.data;
        packInfo={0,0,0,nullptr};
    }
    if(startIndex!=0){
        buf.remove(0,startIndex);
        startIndex=0;
    }
}

}

