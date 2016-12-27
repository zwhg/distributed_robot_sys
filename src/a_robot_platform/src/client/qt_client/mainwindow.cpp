#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <QString>
#include<unistd.h>
#include<string.h>
#include<stdio.h>
#include<math.h>
#include "../../common/modbus.h"
#include "../../common/use_display.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_keyControl(new zw::KeyControl(parent)),
    m_tcpSocketClient(new zw::TcpSocket(parent))

{
    ui->setupUi(this);
    pixmap = new QPixmap(PIXMAP_X,PIXMAP_Y);
    QTimer *timer = new QTimer(this);
    timer->start(1);
    connect(timer,SIGNAL(timeout()),this,SLOT(ShowLaser()));
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
void MainWindow::ShowLaser()
{
    pixmap->fill(Qt::black);
    QPainter painter(pixmap);
    QColor qcolor(108,108,108);
    painter.setPen(qcolor);

    temp_x = 25;
    temp_y = 25;
    painter.drawEllipse(temp_x,temp_y,Mywidth,Myhigh);
    temp_x = Mywidth/2 - (Mywidth-(Mywidth/3))/2 +25;
    temp_y = Myhigh/2 - (Myhigh-(Myhigh/3))/2 +25;
    painter.drawEllipse(temp_x,temp_y,Mywidth-(Mywidth/3),Myhigh-(Myhigh/3));
    temp_x = Mywidth/2 - (Mywidth-2*(Mywidth/3))/2 +25;
    temp_y = Myhigh/2 - (Myhigh-2*(Myhigh/3))/2 +25;
    painter.drawEllipse(temp_x,temp_y,Mywidth-2*(Mywidth/3),Myhigh-2*(Myhigh/3));
    //绘制垂直和水平的线
    QLine qline1(0 +25,Myhigh/2 +25,Mywidth +25,Myhigh/2 +25);
    QLine qline2(Mywidth/2 +25,Myhigh +25,Mywidth/2 +25,0 +25-10);
    painter.drawLine(qline1);
    painter.drawLine(qline2);
    ////////////////绘制斜线////////////////////////////////////////
    // 将画笔中心点移动至(Mywidth/2,Myhigh/2)
    painter.translate(QPoint(Mywidth/2 +25,Myhigh/2 +25));
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(60);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    painter.rotate(30);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    //标记角度和距离
    painter.drawText(0,-Mywidth/6 -5,"833");
    painter.drawText(0,-Mywidth/3 -5,"1666");
    painter.drawText(0,-Mywidth/2 +15,"2500");
    for(unsigned int i=0;i<360;i+=30)
    {
        QString *st = new QString;
        if(i>=270)
        {
            st->setNum(i-270);
        }
        else
        {
            st->setNum(i+90);
        }
        temp_x = cos((i)*CAMBER)*(Mywidth/2+10);
        temp_y = sin((i)*CAMBER)*(Mywidth/2+10);
        if(i<=210)
        {
            if((i<=90)&&(i>0))
            {
                temp_x += 10;
            }
            else
            {
                temp_x -= 10;
            }
            if(i==0)
            {
                temp_x +=5;
            }
            if(i==180)
            {
                temp_x -= 5;
            }
            else if(i==210)
            {
                temp_x -= 5;
                temp_y -=10;
            }
            temp_y += 10;
        }
        painter.drawText(QPoint(temp_x,temp_y),*st);
    }
    //绘制点
    qcolor.setRgb(255,255,240);
   painter.setPen(qcolor);
//    painter.setBrush(Qt::white);
    painter.drawText(-250,-Mywidth/2 +620,"Rotate Speed:");
//    QString txt5 = QString("%1").arg(speed);
//    painter.drawText(-160,-Mywidth/2+620,txt5);
    for(unsigned int y=0;y<MIDD;y++)
    {
         if(getModeSelect.checkedId()==Show_All)
         {
             painter.drawPoint(ShowPoint[y][0],ShowPoint[y][1]);
         }else if(getModeSelect.checkedId()==Show_2M)
         {
             if(distance[y]<=2000)
             {
                 painter.drawPoint(ShowPoint[y][0],ShowPoint[y][1]);
             }
         }else if(getModeSelect.checkedId()==Show_1M)
         {
             if(distance[y]<=1000)
             {
                 painter.drawPoint(ShowPoint[y][0],ShowPoint[y][1]);
             }
         }
    }
    painter.drawText(-250,-Mywidth/2+650,"Forward Width:");
    painter.drawText(-160,-Mywidth/2+650,QString("%1").arg(wid_forward));
    painter.drawText(-60,-Mywidth/2+650,"Distance:");
    painter.drawText(30,-Mywidth/2+650,QString("%1").arg(dis_forward));

    painter.drawText(-250,-Mywidth/2+680,"Back Width:");
    painter.drawText(-160,-Mywidth/2+680,QString("%1").arg(wid_back));
    painter.drawText(-60,-Mywidth/2+680,"Distance:");
    painter.drawText(30,-Mywidth/2+680,QString("%1").arg(dis_back));

    painter.drawText(-250,-Mywidth/2+710,"Left Width:");
    painter.drawText(-160,-Mywidth/2+710,QString("%1").arg(wid_left));
    painter.drawText(-60,-Mywidth/2+710,"Distance:");
    painter.drawText(30,-Mywidth/2+710,QString("%1").arg(dis_left));

    painter.drawText(-250,-Mywidth/2+740,"Right Width:");
    painter.drawText(-160,-Mywidth/2+740,QString("%1").arg(wid_right));
    painter.drawText(-60,-Mywidth/2+740,"Distance:");
    painter.drawText(30,-Mywidth/2+740,QString("%1").arg(dis_right));

    painter.drawText(200,-Mywidth/2+680,"Version : V1.0");
    painter.drawText(200,-Mywidth/2+710,"Author : shenyu");
    painter.drawText(200,-Mywidth/2+740,"Date : 2016-12");
  //draw the arrow of  directiion
    QPolygonF  polygon;
    if(dis_forward <= Safe_Distance)   //Warning!!!
    {
         painter.setBrush(Qt::red);
         painter.drawEllipse(330,-Mywidth/2+85,20,20);
    }
    else if(dis_forward > Safe_Distance && dis_forward < Avoid_Distance)  //avoid obstacle
    {
        painter.setBrush(Qt::yellow);
        painter.drawEllipse(330,-Mywidth/2+35,20,20);
    }
    else
    {
        painter.setBrush(Qt::green);   //normal  run
        painter.drawEllipse(330,-Mywidth/2-15,20,20);
    }

    polygon<<QPointF(0.0,0.0)<<QPointF(-(((Car_R+Safe_Distance)*Myhigh/2)/DIA)*tan(PI/12),-((Safe_Distance+Car_R)*Myhigh/2)/DIA)
                  <<QPointF((((Car_R+Safe_Distance)*Myhigh/2)/DIA)*tan(PI/12),-((Safe_Distance+Car_R)*Myhigh/2)/DIA);
    painter.drawPolygon(polygon,Qt::WindingFill);
  //draw the car outline
    painter.setBrush(Qt::yellow);
    painter.drawEllipse(-((Car_R*Myhigh/2)/DIA),-((Car_R*Myhigh/2)/DIA),(2*Car_R*Myhigh/2)/DIA,(2*Car_R*Myhigh/2)/DIA);
 //draw the laser outline
    painter.setBrush(Qt::black);
    painter.drawEllipse(-((Laser_R*Myhigh/2)/DIA),-((Laser_R*Myhigh/2)/DIA),(2*Laser_R*Myhigh/2)/DIA,(2*Laser_R*Myhigh/2)/DIA);
 // display the state of the car
    painter.drawText(230,-Mywidth/2,"Normal  Run:");
    painter.drawText(230,-Mywidth/2+50,"Avoid Obstacle:");
    painter.drawText(230,-Mywidth/2+100,"Warning:");

    memset(ShowPoint,0,sizeof(ShowPoint));  //reset the result array
    painter.end();
    ui->label_main->setPixmap(*pixmap);
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
