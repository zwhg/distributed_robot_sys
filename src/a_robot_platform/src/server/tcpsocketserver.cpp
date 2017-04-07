#include <qdebug.h>
#include <QtNetwork/QHostAddress>
#include <QTimer>
#include "tcpsocketserver.h"

namespace zw {


static pthread_mutex_t g_tMutex  = PTHREAD_MUTEX_INITIALIZER;


TcpSocketServer::TcpSocketServer()
{
    cmd_time_out = 0 ;
    m_listenSocket =new QTcpServer();
    m_listenSocket->listen(QHostAddress::Any , SOCKET_PORT);
    QObject::connect(this->m_listenSocket,SIGNAL(newConnection()),this,SLOT(on_ProcessConnection()));
    QTimer *timer = new QTimer(this);
    QObject::connect(timer,SIGNAL(timeout()),this,SLOT(cmdTimeOut()));
    timer->start(100);
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
    QByteArray arry= this->m_readWriteSocket->readAll();
    buf.append(arry);

    int32_t endIndex = buf.count();
    if(endIndex >=FIXEDLENGTH){
        int32_t startIndex=0;
        ParaGetSet packInfo={NULL1,0,NULL2,nullptr};
        Paras m_para;

        while(m_modbus.UnPackparas((const byte*)buf.data(),startIndex ,endIndex, packInfo))
        {
            if(packInfo.fuc == R_REGISTER){
                packInfo.data=new int32_t[packInfo.len];
                m_para.GetAddressValue(packInfo);
                packInfo.fuc=W_REGISTER;
                int32_t size=FIXEDLENGTH +packInfo.len*4;
                byte* msg =new byte[size];
                if(size!=m_modbus.PackParas(packInfo,msg))
                    qDebug () <<"Error in pack size!";
                else
                    this->m_readWriteSocket->write((char*)msg ,size);
                delete msg;
                delete packInfo.data;
            }else if(packInfo.fuc == W_REGISTER){
                m_para.SetAddressValue(packInfo);
#if 0
                int32_t dat[2];
                zw::ParaGetSet  pack = {zw::R_REGISTER,2,zw::CONTROL,dat};
                m_para.GetAddressValue(pack);
                zw::Float2Int32 ff1,ff2;
                ff1.i=dat[0];
                ff2.i=dat[1];
                qDebug () <<ff1.f<<ff2.f;
#endif
#if 0
                int32_t dat[1];
                zw::ParaGetSet  pack = {zw::R_REGISTER,1,zw::BTN_SWITCH,dat};
                m_para.GetAddressValue(pack);
                qDebug () <<dat[0];
#endif
                if(packInfo.data!=nullptr)
                {
                   delete packInfo.data;
                   packInfo.data=nullptr;
                }
            }

            {
                pthread_mutex_lock(&g_tMutex );
                cmd_time_out =0;
                pthread_mutex_unlock(&g_tMutex );
            }
        }
        if(startIndex!=0){
            buf.remove(0,startIndex);
            startIndex=0;
        }
    }
}

 void TcpSocketServer::cmdTimeOut()
 {
     int32_t dat[2];
     ParaGetSet pack={zw::R_REGISTER,1, BTN_SWITCH,dat};
     Paras m_para;
     m_para.GetAddressValue(pack);
     if((dat[0]&KEY_VEL_CTR)!=KEY_VEL_CTR)
         return ;

    bool flag =false ;
    pthread_mutex_lock(&g_tMutex );
    cmd_time_out ++ ;
    if(cmd_time_out >5)
    {
        cmd_time_out=0;
        flag =true;
    }
    pthread_mutex_unlock(&g_tMutex);

    if(flag)
    {       
        Float2Int32 f2is,f2io;
        pack = {zw::W_REGISTER,2,zw::CONTROL,dat};
        f2is.f=0;
        f2io.f=0;
        dat[0]=f2is.i;
        dat[1]=f2io.i;
        m_para.SetAddressValue(pack);
        qDebug()<<"tcp time out!";
    }
 }

}

