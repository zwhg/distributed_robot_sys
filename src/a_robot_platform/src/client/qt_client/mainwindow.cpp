#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <QString>
#include "../../common/modbus.h"



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_keyControl(new zw::KeyControl(parent)),
    m_tcpSocketClient(new zw::TcpSocket(parent))

{
    ui->setupUi(this);

    m_tcpSocketClient->host =ui->lEdit_ip->text().toStdString();
    m_tcpSocketClient->port =ui->lEdit_port->text().toUShort();

    QObject::connect(ui->pBtn_start2connect,SIGNAL(clicked(bool)),
                     this,SLOT(on_pBtn_start2connect_clicked(bool)));
    QObject::connect(ui->pBtn_key_control_open,SIGNAL(clicked(bool)),
                     this,SLOT(on_pBtn_key_control_open_clicked(bool)));
    QObject::connect(ui->lEdit_ip,SIGNAL(returnPressed()),
                     this,SLOT(on_lEdit_ip_returnPressed()));
    QObject::connect(ui->lEdit_port,SIGNAL(returnPressed()),
                     this,SLOT(on_lEdit_port_returnPressed()));


    QTimer *x_timer =new QTimer();
    QObject::connect(x_timer,SIGNAL(timeout()),this,SLOT(on_xTimerUpdate()));
    x_timer->start(50);

    QTimer *cmd_timer =new QTimer();
    QObject::connect(cmd_timer,SIGNAL(timeout()),this,SLOT(on_cmdTimerUpdate()));
    cmd_timer->start(40);
}

MainWindow::~MainWindow()
{
    delete m_keyControl;
    delete m_tcpSocketClient;
    this->close();
    delete ui;
}


void MainWindow::on_xTimerUpdate(void)
{
    switch (m_tcpSocketClient->m_connectStatus) {
    case zw::DISCONNECTED:
        ui->pBtn_start2connect->setStyleSheet("background-color: rgb(167, 167, 125);");
        ui->pBtn_start2connect->setText("disconnected");
        break;
    case zw::CONNECTING:
        ui->pBtn_start2connect->setStyleSheet("background-color: rgb(255, 0, 0);");
        ui->pBtn_start2connect->setText("connecting");
        break;
    case zw::CONNECTED:
        ui->pBtn_start2connect->setStyleSheet("background-color: rgb(0, 255, 0);");
        ui->pBtn_start2connect->setText("connected");
        break;
    case zw::LOSECONNECT:
        ui->pBtn_start2connect->setStyleSheet("background-color: rgb(255, 0, 0);");
        ui->pBtn_start2connect->setText("lose connect");
        break;
    }
    KeyControlMsgRefalsh(m_keyControl->kMsg);
}

void MainWindow::on_cmdTimerUpdate(void)
{
    zw::ParaGetSet msgInfo;

    if(m_keyControl->keyControl){
        msgInfo={zw::W_MULTI_REGISTER,2,zw::CONTROL,nullptr};
        m_tcpSocketClient->SendMsg(msgInfo);
    }

    msgInfo={zw::R_HOLDING_REGISTER,5,zw::MSG_CONTROL,nullptr};
    m_tcpSocketClient->SendMsg(msgInfo);

    msgInfo={zw::R_HOLDING_REGISTER,6,zw::MSG_IMU,nullptr};
    m_tcpSocketClient->SendMsg(msgInfo);
}

void MainWindow::KeyControlMsgRefalsh(const zw::KeyControlMsg & kMsg)
{
    ui->lbl_vel_max->setText(QString::number(kMsg.maxSpeed,'f',2));
    ui->lbl_vel_exp->setText(QString::number(kMsg.e_speed,'f',2));
    ui->lbl_vel_ret->setText(QString::number(kMsg.a_speed,'f',2));
    ui->lbl_ome_max->setText(QString::number(kMsg.maxOmega,'f',2));
    ui->lbl_ome_exp->setText(QString::number(kMsg.e_omega,'f',2));
    ui->lbl_ome_ret->setText(QString::number(kMsg.a_omega,'f',2));
}

void MainWindow::MsgControlRefalsh(void)
{
    int32_t dat[2];
    zw::ParaGetSet  packInfo = {zw::R_HOLDING_REGISTER,2,zw::MSG_CONTROL,dat};
    zw::modbus.GetAddressValue(packInfo);
    zw::Float2Int32 ff;
    ff.i=dat[0];
    ui->lbl_vel_ret->setText(QString::number(ff.f,'f',2));
    ff.i=dat[1];
    ui->lbl_ome_ret->setText(QString::number(ff.f,'f',2));
}



void MainWindow::on_pBtn_start2connect_clicked(bool checked)
{
    if(checked)
    {
        if(!m_tcpSocketClient->doConnect){
            m_tcpSocketClient->doConnect =true;
            m_tcpSocketClient->Connect();
        }
    }
    else
    {
        if(m_tcpSocketClient->doConnect){
            m_tcpSocketClient->doConnect =false;
            m_tcpSocketClient->DisConnect();
        }
    }
}

void MainWindow::on_pBtn_key_control_open_clicked(bool checked)
{
    if(checked){
        ui->pBtn_key_control_open->setStyleSheet("background-color: rgb(0, 255, 0);");
        ui->pBtn_key_control_open->setText("started");
         m_keyControl->keyControl =true;
    }
    else{
        ui->pBtn_key_control_open->setStyleSheet("background-color: rgb(167, 167, 125)");
        ui->pBtn_key_control_open->setText("stoped");
        m_keyControl->keyControl =false;
    }
}

void MainWindow::on_lEdit_ip_returnPressed()
{
    m_tcpSocketClient->host =ui->lEdit_ip->text().toStdString();
}

void MainWindow::on_lEdit_port_returnPressed()
{
    m_tcpSocketClient->port =ui->lEdit_port->text().toUShort();
}

void MainWindow::keyPressEvent(QKeyEvent *e)
{
    if(m_keyControl->keyControl)
        m_keyControl->ControlKeyDown(e);
}
