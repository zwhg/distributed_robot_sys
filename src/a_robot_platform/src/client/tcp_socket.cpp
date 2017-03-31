#include "tcp_socket.h"
#include "../client/tcp_socket.h"
#include <qdebug.h>
#include <QException>

namespace zw{
    TcpSocket::TcpSocket(QObject *parent):
        QObject(parent)
    {
        m_tcpClient= new QTcpSocket();
        doConnect=false;
        isConnected =false;
        m_connectStatus=DISCONNECTED;

        QObject::connect(m_tcpClient, SIGNAL(connected()),this, SLOT(on_Connected()));
        QObject::connect(m_tcpClient, SIGNAL(disconnected()),this, SLOT(on_Disconnected()));
        QObject::connect(m_tcpClient, SIGNAL(readyRead()),this, SLOT(on_ReadyRead()));
    }

    TcpSocket:: ~TcpSocket()
    {
        delete m_tcpClient;
    }

    bool TcpSocket::Connect(void)
    {
        DoConnect(host ,port);
        uint32_t timeout=0;
        while(!IsConnect()){
            timeout ++;
            DoConnect(host,port);
            if(timeout>CONNECT_TIMEOUT){
                m_connectStatus = LOSECONNECT;
                return false;
            }
        }
        return true;
    }

    void TcpSocket::DisConnect(void)
    {
        if(IsConnect())
        {
            m_tcpClient->disconnectFromHost();
      //      m_tcpClient->waitForDisconnected();
        }
        m_connectStatus =  DISCONNECTED;
    }

    void TcpSocket::DoConnect(std::string host ,uint16_t port)
    {

        qDebug() <<"connecting to " << QString::fromStdString(host)<< ":" <<port;

        // this is not blocking call
        m_tcpClient->connectToHost(QString::fromStdString(host), port);

        m_connectStatus =  CONNECTING;

         // we need to wait...
        if(! m_tcpClient->waitForConnected(1000))
        {
            qDebug () << "Error: " << m_tcpClient->errorString();
            return ;
        }
        isConnected =true;
        m_connectStatus =CONNECTED;
    }

    void TcpSocket::on_Connected(void)
    {
        qDebug() << "connected...";
    }

    void TcpSocket::on_Disconnected(void)
    {
        qDebug() << "disconnected...";
        isConnected =false;
        if(doConnect)
            Connect();
    }

    bool TcpSocket::IsConnect(void)
    {
        //return m_tcpClient->isOpen();
        return isConnected;
    }

    void TcpSocket::on_ReadyRead(void)
    {
        QByteArray arry= m_tcpClient->readAll();
        buf.append(arry);

        int32_t endIndex = buf.count();
        if(endIndex >=FIXEDLENGTH){
            int32_t startIndex=0;
            ParaGetSet  packInfo = {NULL1,0,NULL2,nullptr};
            Paras m_para;

            while(m_modbus.UnPackparas((const byte*)buf.data(),startIndex ,endIndex, packInfo))
            {

                if(packInfo.fuc == R_REGISTER){
                    packInfo.data = new int32_t[packInfo.len];
                    m_para.GetAddressValue(packInfo);
                    packInfo.fuc=W_REGISTER;
                    int32_t size=FIXEDLENGTH +packInfo.len*4;
                    byte* msg =new byte[size];
                    if(size!=m_modbus.PackParas(packInfo,msg))
                        qDebug()<<"Error in pack size!";
                    else
                        SendMsg(msg ,size);
                    delete msg;
                    delete packInfo.data;
                }else if(packInfo.fuc == W_REGISTER){
                    m_para.SetAddressValue(packInfo);
                   // qDebug()<<"read success!";
                    if(packInfo.data!=nullptr)
                    {
                       delete packInfo.data;
                       packInfo.data=nullptr;
                    }
                }
            }
            if(startIndex!=0){
                buf.remove(0,startIndex);
                startIndex=0;
            }
        }
    }

    bool TcpSocket::SendMsg(const byte *bytes,uint16_t size)
    {
        if(IsConnect()){
            try{
                m_tcpClient->write((char *)bytes,size);
            }catch(QException &e){
                qDebug()<<e.what();
                return false;
            }
           return true;
        }
        return false;
    }

    bool TcpSocket::SendMsg(ParaGetSet &packInfo)
    {
        bool res =false;
        int32_t size;
        byte *msg;
        Paras m_para;
        if(packInfo.fuc == R_REGISTER){
            size=FIXEDLENGTH;
        }else if(packInfo.fuc == W_REGISTER){
            packInfo.data = new int32_t[packInfo.len];
            if(!m_para.GetAddressValue(packInfo))
                return res;
            size=FIXEDLENGTH +packInfo.len*4;
        }else
            return res;
        msg =new byte[size];
        if(size!=m_modbus.PackParas(packInfo,msg)){
            qDebug()<<"Error in pack size!";
        }else{
            res=SendMsg(msg ,size);
        }
        delete msg;
        if(packInfo.fuc == W_REGISTER)
        {
            if(packInfo.addr==BTN_SWITCH)
            {
            //    qDebug("%d",packInfo.data[0]);
                m_para.ResetKeyRegister();
            }
            delete packInfo.data;
        }
     return res;
    }
}
