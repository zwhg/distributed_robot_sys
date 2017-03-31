#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include<QVector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../../common/modbus.h"
#include "../../common/use_display.h"
#include "../../common/paras.h"
#include "../../common/map_image.h"
#include "../udp_socket.h"


#include <QtNetwork>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_keyControl(new zw::KeyControl(parent)),
    m_tcpSocketClient(new zw::TcpSocket(parent)),
    m_udpSocketClient(new zw::UdpSocket(parent))
{
    ui->setupUi(this);
    pixmap = new QPixmap(PIXMAP_X,PIXMAP_Y);
    QTimer *timer = new QTimer(this);
    timer->start(50);
    connect(timer,SIGNAL(timeout()),this,SLOT(ShowLaser()));
    connect(timer,SIGNAL(timeout()),this,SLOT(ShowUltrasonic()));

    ui->lEdit_ip->setText(QString::fromStdString(zw::SERVER_IP));
    m_tcpSocketClient->host =ui->lEdit_ip->text().toStdString();
    m_tcpSocketClient->port =ui->lEdit_port->text().toUShort();

    QTimer *x_timer =new QTimer();
    QObject::connect(x_timer,SIGNAL(timeout()),this,SLOT(xTimerUpdate()));
    x_timer->start(50);

    QTimer *cmd_timer =new QTimer();
    QObject::connect(cmd_timer,SIGNAL(timeout()),this,SLOT(cmdTimerUpdate()));
    cmd_timer->start(40);

    img_binarization=false;
    th_binarization=127;
}

MainWindow::~MainWindow()
{
    delete m_keyControl;
    delete m_tcpSocketClient;
    delete m_udpSocketClient;
    this->close();
    delete ui;
}

void MainWindow::ShowUltrasonic()   //显示超声
{
    pixmap->fill(Qt::white);
    QPainter painter(pixmap);
//draw  the ultrasonic  Area
      painter.setBrush(Qt::gray);
      QPolygonF  polygon1,polygon2,polygon3,polygon4,polygon5,polygon6,polygon7,polygon8;
      polygon1<<QPointF(ONE_Ultra_X,ONE_Ultra_Y)<<QPointF(ONE_Ultra_X+dis[0],ONE_Ultra_Y-dis[0]*tan(PI/12))
                      <<QPointF(ONE_Ultra_X+dis[0],ONE_Ultra_Y+dis[0]*tan(PI/12));
      polygon2<<QPointF(TWO_Ultra_X,TWO_Ultra_Y)<<QPointF(TWO_Ultra_X+dis[1],TWO_Ultra_Y-dis[1]*tan(PI/12))
                      <<QPointF(TWO_Ultra_X+dis[1],TWO_Ultra_Y+dis[1]*tan(PI/12));
      polygon3<<QPointF(THREE_Ultra_X,THREE_Ultra_Y)
                      <<QPointF(THREE_Ultra_X+(dis[2]/cos(PI/12))*cos(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))-PI/12),
                                         THREE_Ultra_Y- (dis[2]/cos(PI/12))*sin(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))-PI/12))
                      <<QPointF(THREE_Ultra_X+(dis[2]/cos(PI/12))*cos(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))+PI/12),
                                         THREE_Ultra_Y- (dis[2]/cos(PI/12))*sin(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))+PI/12));
      polygon4<<QPointF(FOUR_Ultra_X,FOUR_Ultra_Y)<<QPointF(FOUR_Ultra_X-dis[3]*tan(PI/12),FOUR_Ultra_Y-dis[3])
                     <<QPointF(FOUR_Ultra_X+dis[3]*tan(PI/12),FOUR_Ultra_Y-dis[3]);
      polygon5<<QPointF(FIVE_Ultra_X,FIVE_Ultra_Y)<<QPointF(FIVE_Ultra_X-dis[4]*tan(PI/12),FIVE_Ultra_Y-dis[4])
                     <<QPointF(FIVE_Ultra_X+dis[4]*tan(PI/12),FIVE_Ultra_Y-dis[4]);
      polygon6<<QPointF(SIX_Ultra_X,SIX_Ultra_Y)
                      <<QPointF(SIX_Ultra_X-(dis[5]/cos(PI/12))*cos(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))+PI/12),
                                         SIX_Ultra_Y-(dis[5]/cos(PI/12))*sin(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))+PI/12))
                      <<QPointF(SIX_Ultra_X-dis[5]/cos(PI/12)*cos(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))-PI/12),
                                         SIX_Ultra_Y-(dis[5]/cos(PI/12))*sin(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))-PI/12));
      polygon7<<QPointF(SEVEN_Ultra_X,SEVEN_Ultra_Y)<<QPointF(SEVEN_Ultra_X-dis[6],SEVEN_Ultra_Y-dis[6]*tan(PI/12))
                    <<QPointF(SEVEN_Ultra_X-dis[6],SEVEN_Ultra_Y+dis[6]*tan(PI/12));
      polygon8<<QPointF(EIGHT_Ultra_X,EIGHT_Ultra_Y)<<QPointF(EIGHT_Ultra_X-dis[7],EIGHT_Ultra_Y-dis[7]*tan(PI/12))
                     <<QPointF(EIGHT_Ultra_X-dis[7],EIGHT_Ultra_Y+dis[7]*tan(PI/12));
      if(ui->ultra1->isChecked())
      {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon1,Qt::WindingFill);
          }
          ui->lb_ultra1->setText(QString("%1").arg(dis[0]));
      }else
      {
          ui->lb_ultra1->setText("0000");
      }
/**********************1#******************************/
      if(ui->ultra2->isChecked())
      {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon2,Qt::WindingFill);
          }
         ui->lb_ultra2->setText(QString("%1").arg(dis[1]));
      }else
      {
          ui->lb_ultra2->setText("0000");
      }
/**********************2#******************************/
      if(ui->ultra3->isChecked())
      {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon3,Qt::WindingFill);
          }
           ui->lb_ultra3->setText(QString("%1").arg(dis[2]));
      }else
      {
          ui->lb_ultra3->setText("0000");
      }
/**********************3#******************************/
      if(ui->ultra4->isChecked())
      {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon4,Qt::WindingFill);
          }
          ui->lb_ultra4->setText(QString("%1").arg(dis[3]));
      }else
      {
          ui->lb_ultra4->setText("0000");
      }
/**********************4#******************************/
      if(ui->ultra5->isChecked())
      {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon5,Qt::WindingFill);
          }
         ui->lb_ultra5->setText(QString("%1").arg(dis[4]));
      }else
      {
          ui->lb_ultra5->setText("0000");
      }
/**********************5#******************************/
      if(ui->ultra6->isChecked())
      {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon6,Qt::WindingFill);
          }
          ui->lb_ultra6->setText(QString("%1").arg(dis[5]));
      }else
      {
          ui->lb_ultra6->setText("0000");
      }
/**********************6#******************************/
      if(ui->ultra7->isChecked())
      {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon7,Qt::WindingFill);
          }
          ui->lb_ultra7->setText(QString("%1").arg(dis[6]));
      }else
      {
          ui->lb_ultra7->setText("0000");
      }
/**********************7#******************************/
      if(ui->ultra8->isChecked())
      {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon8,Qt::WindingFill);
          }
          ui->lb_ultra8->setText(QString("%1").arg(dis[7]));
      }else
      {
          ui->lb_ultra8->setText("0000");
      }
/**********************8#******************************/
      //draw the obstacle outline
          QPointF points[8] = {
              QPointF(ONE_Ultra_X + dis[0],ONE_Ultra_Y),
              QPointF(TWO_Ultra_X + dis[1],TWO_Ultra_Y),
              QPointF(THREE_Ultra_X +(((dis[2]/cos(PI/12))*cos(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))-PI/12))+((dis[2]/cos(PI/12))*cos(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))+PI/12)))/2.0,
              THREE_Ultra_Y-(((dis[2]/cos(PI/12))*sin(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))-PI/12))+((dis[2]/cos(PI/12))*sin(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))+PI/12)))/2.0),
              QPointF(FOUR_Ultra_X,FOUR_Ultra_Y-dis[3]),
              QPointF(FIVE_Ultra_X,FIVE_Ultra_Y-dis[4]),
              QPointF(SIX_Ultra_X-(((dis[5]/cos(PI/12))*cos(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))+PI/12))+(dis[5]/cos(PI/12)*cos(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))-PI/12)))/2.0,
              SIX_Ultra_Y-(((dis[5]/cos(PI/12))*sin(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))+PI/12))+((dis[5]/cos(PI/12))*sin(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))-PI/12)))/2.0),
              QPointF(SEVEN_Ultra_X-dis[6],SEVEN_Ultra_Y),
              QPointF(EIGHT_Ultra_X-dis[7],EIGHT_Ultra_Y)};
              painter.setPen(QPen(Qt::red,2,Qt::DashDotLine,Qt::RoundCap));
               if(ui->Obsta_OutLine->isChecked())
                    painter.drawPolyline(points,8);
      painter.setPen(QPen(Qt::black,1,Qt::DashDotLine,Qt::RoundCap));        //set style of the pen
      //绘制垂直和水平的虚线
      for(int i=0;i<=PIXMAP_Y;i=i+ONE_GRID)
      {
          painter.drawLine(0,i,PIXMAP_X,i);
      }
      for(int j=0;j<=PIXMAP_X;j=j+ONE_GRID)
      {
          painter.drawLine(j,0,j,PIXMAP_Y);
      }
      //绘制坐标轴
       painter.setPen(QPen(Qt::green,2,Qt::SolidLine,Qt::RoundCap)); //set style of the pen
       painter.drawLine(0,PIXMAP_Y,PIXMAP_X,PIXMAP_Y);
       painter.drawLine(7*ONE_GRID,0,7*ONE_GRID,PIXMAP_Y);
      //draw the car outline
        painter.setBrush(Qt::red);
        painter.drawEllipse(5.425*ONE_GRID,10.425*ONE_GRID,3.15*ONE_GRID,3.15*ONE_GRID);
     //draw the laser outline
        painter.setBrush(Qt::black);
        painter.drawEllipse(6.565*ONE_GRID,11.565*ONE_GRID,0.87*ONE_GRID,0.87*ONE_GRID);
        ui->label_Ultra->setPixmap(*pixmap);
}

void MainWindow::ShowLaser()
{
//get the laser distance
    for(int i=0;i < TAG;i++)
    {
      //  qDebug()<<"laser_dis:"<<laser_dis[i];
        distance[i] = laser_dis[i];
        tem_y = -laser_dis[i]*cos(0.5*i*CAMBER);
        tem_x = laser_dis[i]*sin(0.5*i*CAMBER);
        ShowPoint[i][0] = (tem_x*(Myhigh/2))/DIA;
        ShowPoint[i][1] = (tem_y*(Myhigh/2))/DIA;
//qDebug()<<"(x,y):"<<"("<<ShowPoint[i][0]<<","<<ShowPoint[i][1]<<")";
    }

    pixmap->fill(Qt::black);
    QPainter painter(pixmap);
    QColor qcolor(108,108,108);
    painter.setPen(qcolor);
    getModeSelect.addButton(ui->rb_all,Show_All);
    getModeSelect.addButton(ui->rb_2m,Show_2M);
    getModeSelect.addButton(ui->rb_1m,Show_1M);

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
    /////////////////////////绘制斜线///////////////////////////
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
//     painter.setBrush(Qt::white);
    painter.drawText(-250,-Mywidth/2 +620,"Rotate Speed:");
//     QString txt5 = QString("%1").arg(speed);
//     painter.drawText(-160,-Mywidth/2+620,txt5);
    for(unsigned int y=0;y<MIDD;y++)
    {
         if(getModeSelect.checkedId()==Show_All)
         {
             painter.drawPoint(ShowPoint[y][0],ShowPoint[y][1]);
         }else if(getModeSelect.checkedId()==Show_2M)
         {
             if(distance[y]<=Set_First_Limit)
             {
                 painter.drawPoint(ShowPoint[y][0],ShowPoint[y][1]);
             }
         }else if(getModeSelect.checkedId()==Show_1M)
         {
             if(distance[y]<=Set_Second_Limit)
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

    painter.drawText(200,-Mywidth/2+680,"Version : V2.0");
    painter.drawText(200,-Mywidth/2+710,"Author : shenyu");
    painter.drawText(200,-Mywidth/2+740,"Date : 2017-01");
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

void MainWindow::xTimerUpdate(void)
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
    MsgImuRefalsh();
    MsgControlRefalsh();
    MsgUltrasonicRefalsh();
}

void MainWindow::cmdTimerUpdate(void)
{
    zw::ParaGetSet msgInfo;

    if(m_keyControl->keyControl){
        msgInfo={zw::W_REGISTER,2,zw::CONTROL,nullptr};
        m_tcpSocketClient->SendMsg(msgInfo);     
    }   

    msgInfo={zw::R_REGISTER,5,zw::MSG_CONTROL,nullptr};
    m_tcpSocketClient->SendMsg(msgInfo);

    msgInfo={zw::R_REGISTER,6,zw::MSG_IMU,nullptr};
    m_tcpSocketClient->SendMsg(msgInfo);

    msgInfo={zw::R_REGISTER,8,zw::MSG_Ultrasonic,nullptr};
    m_tcpSocketClient->SendMsg(msgInfo);
}

void MainWindow::KeyControlMsgRefalsh(const zw::KeyControlMsg & kMsg)
{
    ui->lbl_vel_max->setText(QString::number(kMsg.maxSpeed,'f',2));
    ui->lbl_vel_exp->setText(QString::number(kMsg.e_speed,'f',2));
   // ui->lbl_vel_ret->setText(QString::number(kMsg.a_speed,'f',2));
    ui->lbl_ome_max->setText(QString::number(kMsg.maxOmega,'f',2));
    ui->lbl_ome_exp->setText(QString::number(kMsg.e_omega,'f',2));
   // ui->lbl_ome_ret->setText(QString::number(kMsg.a_omega,'f',2));
}

void MainWindow::MsgControlRefalsh(void)
{
    zw::Paras m_para;
    int32_t dat[5];
    zw::ParaGetSet  packInfo = {zw::R_REGISTER,5,zw::MSG_CONTROL,dat};
    m_para.GetAddressValue(packInfo);
    zw::Float2Int32 ff;
    ff.i=dat[0];
    ui->lbl_vel_ret->setText(QString::number(ff.f,'f',2));
    ff.i=dat[1];
    ui->lbl_ome_ret->setText(QString::number(ff.f,'f',2));
    ff.i=dat[2];
    ui->lbl_pose_x->setText(QString::number(ff.f,'f',2));
    ff.i=dat[3];
    ui->lbl_pose_y->setText(QString::number(ff.f,'f',2));
    ff.i=dat[4];
    ui->lbl_pose_h->setText(QString::number(ff.f,'f',2));
}

void MainWindow::MsgImuRefalsh(void)
{
    zw::Paras m_para;
    int32_t dat[6];
    zw::ParaGetSet  packInfo = {zw::R_REGISTER,6,zw::MSG_IMU,dat};
    m_para.GetAddressValue(packInfo);
    ui->lbl_acc_x->setText(QString::number(dat[0]*Acc_Mss,'f',4));
    ui->lbl_acc_y->setText(QString::number(dat[1]*Acc_Mss,'f',4));
    ui->lbl_acc_z->setText(QString::number(dat[2]*Acc_Mss,'f',4));
    ui->lbl_gyr_x->setText(QString::number(dat[3]*Gyro_Gr,'f',4));
    ui->lbl_gyr_y->setText(QString::number(dat[4]*Gyro_Gr,'f',4));
    ui->lbl_gyr_z->setText(QString::number(dat[5]*Gyro_Gr,'f',4));
}

void MainWindow::MsgUltrasonicRefalsh(void)
{
    zw::Paras m_para;
    int32_t dat[Ultra_Num];
    zw::Float2Int32  fi;
    zw::ParaGetSet packInfo = {zw::R_REGISTER,Ultra_Num,zw::MSG_Ultrasonic,dat};
    m_para.GetAddressValue(packInfo);
    for(int i=0;i<Ultra_Num;i++)
    {
        fi.i = dat[i];
        dis[i] = fi.f;
    //    qDebug()<<"ultrasonic data:"<<dis[i];
    }
}

void MainWindow::on_pBtn_start2connect_clicked(bool checked)
{
    if(checked)
    {
        if(!m_tcpSocketClient->doConnect){
            m_tcpSocketClient->doConnect =true;
            m_tcpSocketClient->Connect();
        }
    }else{
        if(m_tcpSocketClient->doConnect){
            m_tcpSocketClient->doConnect =false;
            m_tcpSocketClient->DisConnect();
        }
    }
}

void MainWindow::on_pBtn_key_control_open_clicked(bool checked)
{
    int32_t dat[1];
    zw::ParaGetSet packInfo={zw::R_REGISTER,1, zw::BTN_SWITCH,dat};
    zw::Paras m_para;
    m_para.GetAddressValue(packInfo);

    if(checked){
        ui->pBtn_key_control_open->setStyleSheet("background-color: rgb(0, 255, 0);");
        ui->pBtn_key_control_open->setText("started");
         m_keyControl->keyControl =true;
         dat[0]|=KEY_VEL_CTR;
    }else{
        ui->pBtn_key_control_open->setStyleSheet("background-color: rgb(167, 167, 125)");
        ui->pBtn_key_control_open->setText("stoped");
        m_keyControl->keyControl =false;
        dat[0] &=(~KEY_VEL_CTR);
    }
    packInfo.fuc =zw::W_REGISTER;
    m_para.SetAddressValue(packInfo);

    m_tcpSocketClient->SendMsg(packInfo);
}

void MainWindow::on_pBtn_key_Init_IMU_clicked()
{
    int32_t dat[1];
    zw::ParaGetSet packInfo={zw::R_REGISTER,1, zw::BTN_SWITCH,dat};
    zw::Paras m_para;
    m_para.GetAddressValue(packInfo);
    dat[0]|=KEY_INIT_IMU;
    packInfo.fuc =zw::W_REGISTER;
    m_para.SetAddressValue(packInfo);

    m_tcpSocketClient->SendMsg(packInfo);
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

void MainWindow::on_ultraAll_clicked()
{
    if(ui->ultraAll->isChecked())
    {
        ui->ultra1->setChecked(true);
        ui->ultra2->setChecked(true);
        ui->ultra3->setChecked(true);
        ui->ultra4->setChecked(true);
        ui->ultra5->setChecked(true);
        ui->ultra6->setChecked(true);
        ui->ultra7->setChecked(true);
        ui->ultra8->setChecked(true);
    }else{
        ui->ultra1->setChecked(false);
        ui->ultra2->setChecked(false);
        ui->ultra3->setChecked(false);
        ui->ultra4->setChecked(false);
        ui->ultra5->setChecked(false);
        ui->ultra6->setChecked(false);
        ui->ultra7->setChecked(false);
        ui->ultra8->setChecked(false);
    }
}

void MainWindow::on_pBtn_open_map_clicked()
{
    using namespace cv;
    QImage img ;
    QString img_name = QFileDialog::getOpenFileName(this,tr("Open Image"),".",tr("Image Files(*.png *.jpg *.pgm *.bmp)"));
    map=imread(img_name.toLatin1().data());
    m_mapImage.GetQImage(map,img);
    ui->lbl_map->setPixmap(QPixmap::fromImage(img));
    ui->lbl_map->resize(img.width(),img.height());
    ui->lbl_map->setScaledContents(true);

    if((!map.empty())&&(!submap.empty()))
        m_mapImage.SurfFeatureMatch(map,submap);
}

void MainWindow::on_pBtn_open_submap_clicked()
{
    using namespace cv;
    QImage img ;
    QString img_name = QFileDialog::getOpenFileName(this,tr("Open Image"),".",tr("Image Files(*.png *.jpg *.pgm *.bmp)"));
    submap=imread(img_name.toLatin1().data());
//    m_mapImage.GetQImage(submap,img);
//    ui->lbl_sub_map->setPixmap(QPixmap::fromImage(img));
//    ui->lbl_sub_map->resize(img.width(),img.height());
//    ui->lbl_sub_map->setScaledContents(true);

    if((!map.empty())&&(!submap.empty()))
        m_mapImage.SurfFeatureMatch(map,submap);
  //  m_mapImage.OrbFeaturematch(map,submap);
  //  m_mapImage.SiftFeaturematch(map,submap);
}

void MainWindow::on_pBtn_binarization_clicked(bool checked)
{
    if(checked)
    {
        img_binarization=true;
        ui->pBtn_binarization->setStyleSheet("background-color: rgb(0, 255, 0);");
    }
    else
    {
        img_binarization=false;
        ui->pBtn_binarization->setStyleSheet("background-color: rgb(167, 167, 125)");
    }
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    if(img_binarization)
    {
        th_binarization=value;
    }
    QString str="二值化:";
    str += QString::number(th_binarization);
    ui->pBtn_binarization->setText(str);
    ui->horizontalSlider->setValue(th_binarization);

    m_mapImage.ShowBinaryImage("sample_map",map,th_binarization);
 //   m_mapImage.ShowBinaryImage("sample_submap",submap,th_binarization);
}

void MainWindow::on_Spin_Sample_Count_valueChanged(int arg1)
{
    m_mapImage.sample_cnt=arg1;

    m_mapImage.ShowBinaryImage("sample_map",map,th_binarization);
 //   m_mapImage.ShowBinaryImage("sample_submap",submap,th_binarization);
}

void MainWindow::on_Spin_Filter_Count_valueChanged(int arg1)
{
    m_mapImage.filter_cnt=arg1;
    m_mapImage.ShowBinaryImage("sample_map",map,th_binarization);
  //  m_mapImage.ShowBinaryImage("sample_submap",submap,th_binarization);
}
