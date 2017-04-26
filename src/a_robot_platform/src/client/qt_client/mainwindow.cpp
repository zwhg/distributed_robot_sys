#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <QString>
#include <QFileDialog>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <QVector>
#include <QTextStream>
#include <QFile>
#include <QtNetwork>
#include <QRegExp>
#include <QStringList>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../../common/modbus.h"
#include "../../common/use_display.h"
#include "../../common/paras.h"
#include "../../common/map_image.h"
#include "../udp_socket.h"


QString ins_path ="../ins.txt";
QString ins_Head ="inspection";

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_keyControl(new zw::KeyControl(parent)),
    m_tcpSocketClient(new zw::TcpSocket(parent)),
    m_udpSocketClient(new zw::UdpSocket(parent))
{
    ui->setupUi(this);
    pixmap = new QPixmap(PIXMAP_X,PIXMAP_Y);

    ui->lEdit_ip->setText(QString::fromStdString(zw::SERVER_IP));
    m_tcpSocketClient->host =ui->lEdit_ip->text().toStdString();
    m_tcpSocketClient->port =ui->lEdit_port->text().toUShort();

    QTimer *x_timer =new QTimer();
    x_timer->start(50);
    QObject::connect(x_timer,SIGNAL(timeout()),this,SLOT(xTimerUpdate()));

    QTimer *cmd_timer =new QTimer();
    cmd_timer->start(40);
    QObject::connect(cmd_timer,SIGNAL(timeout()),this,SLOT(cmdTimerUpdate()));

    nav_timer=new QTimer();
    nav_timer->start(50);
    nav_timer->stop();
    QObject::connect(nav_timer,SIGNAL(timeout()),this,SLOT(naviTimerupdate()));

    img_binarization=false;
    save_pose_file =false;
    clear_pose_file =false;
    plot_g_navi =false;
    th_binarization=127;
    mapInfo={0,0,0,0,0,0,0};
    carInfo ={0,0,0,0,0};

    zw::Paras m_para;
    int32_t dat[6];
    zw::ParaGetSet  packInfo = {zw::R_REGISTER,6,(zw::ParaAddress)(zw::ADD_PID+6),dat};
    m_para.GetAddressValue(packInfo);
    ui->let_pose_pid_P->setText(QString::number((uint)dat[0]));
    ui->let_pose_pid_I->setText(QString::number((uint)dat[1]));
    ui->let_pose_pid_D->setText(QString::number((uint)dat[2]));
    ui->let_angle_pid_P->setText(QString::number((uint)dat[3]));
    ui->let_angle_pid_I->setText(QString::number((uint)dat[4]));
    ui->let_angle_pid_D->setText(QString::number((uint)dat[5]));

    packInfo.len =2;
    packInfo.addr =zw::ADD_ERR;
    m_para.GetAddressValue(packInfo);
    ui->lbl_robot_epose_err_x->setText(QString::number(dat[0]/1000.0,'f',3));
    ui->lbl_robot_epose_err_y->setText(QString::number(dat[0]/1000.0,'f',3));
    ui->lbl_robot_epose_err_h->setText(QString::number(dat[1]/1000.0,'f',3));

    navi.EndNodeDisErr =ui->lbl_robot_epose_err_x->text().toFloat();
    navi.EndNodeAngErr =ui->lbl_robot_epose_err_h->text().toFloat();
}

MainWindow::~MainWindow()
{
    delete m_keyControl;
    delete m_tcpSocketClient;
    delete m_udpSocketClient;
    this->close();
    delete ui;
}

void MainWindow::xTimerUpdate(void)
{
    int  index =ui->tabMain->currentIndex();
    switch(index)
    {
    case 0:
        ConnectStatus();
        KeyControlMsgRefresh(m_keyControl->kMsg);
        MsgImuRefresh();
        MsgControlRefresh();
        break;
    case 1:
        PoseRefresh();
        if(img_binarization | navi.g.changed |plot_g_navi)
           redrawMap();
        break;
    case 2:
        ShowLaser();
        break;
    case 3:
        ShowUltrasonic();
        break;
    default:
        break;
    }
    SavePoseFiile();
}

void MainWindow::ConnectStatus(void)
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

void MainWindow::KeyControlMsgRefresh(const zw::KeyControlMsg & kMsg)
{
    ui->lbl_vel_max->setText(QString::number(kMsg.maxSpeed,'f',2));
    ui->lbl_vel_exp->setText(QString::number(kMsg.e_speed,'f',2));
   // ui->lbl_vel_ret->setText(QString::number(kMsg.a_speed,'f',2));
    ui->lbl_ome_max->setText(QString::number(kMsg.maxOmega,'f',2));
    ui->lbl_ome_exp->setText(QString::number(kMsg.e_omega,'f',2));
   // ui->lbl_ome_ret->setText(QString::number(kMsg.a_omega,'f',2));
}

void MainWindow::MsgControlRefresh(void)
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
    ui->lbl_pose_x->setText(QString::number(ff.f,'f',3));
    ff.i=dat[3];
    ui->lbl_pose_y->setText(QString::number(ff.f,'f',3));
    ff.i=dat[4];
    ui->lbl_pose_h->setText(QString::number(ff.f,'f',3));
}

void MainWindow::MsgImuRefresh(void)
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

void MainWindow::MsgUltrasonicRefresh(void)
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

void MainWindow::ShowUltrasonic()   //显示超声
{
    pixmap->fill(Qt::white);
    QPainter painter(pixmap);
//draw  the ultrasonic  Area
    painter.setBrush(Qt::gray);
    QPolygonF  polygon1,polygon2,polygon3,polygon4,polygon5,polygon6,polygon7,polygon8;
    polygon1<<QPointF(ONE_Ultra_X,ONE_Ultra_Y)
            <<QPointF(ONE_Ultra_X+dis[0],ONE_Ultra_Y-dis[0]*tan(PI/12))
            <<QPointF(ONE_Ultra_X+dis[0],ONE_Ultra_Y+dis[0]*tan(PI/12));

    polygon2<<QPointF(TWO_Ultra_X,TWO_Ultra_Y)
            <<QPointF(TWO_Ultra_X+dis[1],TWO_Ultra_Y-dis[1]*tan(PI/12))
            <<QPointF(TWO_Ultra_X+dis[1],TWO_Ultra_Y+dis[1]*tan(PI/12));

    polygon3<<QPointF(THREE_Ultra_X,THREE_Ultra_Y)
            <<QPointF(THREE_Ultra_X+(dis[2]/cos(PI/12))*cos(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))-PI/12),
                      THREE_Ultra_Y-(dis[2]/cos(PI/12))*sin(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))-PI/12))
            <<QPointF(THREE_Ultra_X+(dis[2]/cos(PI/12))*cos(atan((Car_Central_Y-THREE_Ultra_Y)/(THREE_Ultra_X-Car_Central_X))+PI/12),
                      THREE_Ultra_Y- (dis[2]/cos(PI/12))*sin(atan((Car_Central_Y-THREE_Ultra_Y)/ (THREE_Ultra_X-Car_Central_X))+PI/12));

    polygon4<<QPointF(FOUR_Ultra_X,FOUR_Ultra_Y)
            <<QPointF(FOUR_Ultra_X-dis[3]*tan(PI/12),FOUR_Ultra_Y-dis[3])
            <<QPointF(FOUR_Ultra_X+dis[3]*tan(PI/12),FOUR_Ultra_Y-dis[3]);

    polygon5<<QPointF(FIVE_Ultra_X,FIVE_Ultra_Y)
            <<QPointF(FIVE_Ultra_X-dis[4]*tan(PI/12),FIVE_Ultra_Y-dis[4])
            <<QPointF(FIVE_Ultra_X+dis[4]*tan(PI/12),FIVE_Ultra_Y-dis[4]);

    polygon6<<QPointF(SIX_Ultra_X,SIX_Ultra_Y)
            <<QPointF(SIX_Ultra_X-(dis[5]/cos(PI/12))*cos(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))+PI/12),
                      SIX_Ultra_Y-(dis[5]/cos(PI/12))*sin(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))+PI/12))
            <<QPointF(SIX_Ultra_X-dis[5]/cos(PI/12)*cos(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))-PI/12),
                      SIX_Ultra_Y-(dis[5]/cos(PI/12))*sin(atan((SIX_Ultra_Y-Car_Central_Y)/(SIX_Ultra_X-Car_Central_X))-PI/12));

     polygon7<<QPointF(SEVEN_Ultra_X,SEVEN_Ultra_Y)
             <<QPointF(SEVEN_Ultra_X-dis[6],SEVEN_Ultra_Y-dis[6]*tan(PI/12))
             <<QPointF(SEVEN_Ultra_X-dis[6],SEVEN_Ultra_Y+dis[6]*tan(PI/12));

     polygon8<<QPointF(EIGHT_Ultra_X,EIGHT_Ultra_Y)
             <<QPointF(EIGHT_Ultra_X-dis[7],EIGHT_Ultra_Y-dis[7]*tan(PI/12))
             <<QPointF(EIGHT_Ultra_X-dis[7],EIGHT_Ultra_Y+dis[7]*tan(PI/12));

     if(ui->ultra1->isChecked()) {
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon1,Qt::WindingFill);
          }
          ui->lb_ultra1->setText(QString("%1").arg(dis[0]));
      }else{
          ui->lb_ultra1->setText("0000");
      }
/**********************1#******************************/
      if(ui->ultra2->isChecked()){
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon2,Qt::WindingFill);
          }
         ui->lb_ultra2->setText(QString("%1").arg(dis[1]));
      }else{
          ui->lb_ultra2->setText("0000");
      }
/**********************2#******************************/
      if(ui->ultra3->isChecked()){
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon3,Qt::WindingFill);
          }
           ui->lb_ultra3->setText(QString("%1").arg(dis[2]));
      }else{
          ui->lb_ultra3->setText("0000");
      }
/**********************3#******************************/
      if(ui->ultra4->isChecked()){
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon4,Qt::WindingFill);
          }
          ui->lb_ultra4->setText(QString("%1").arg(dis[3]));
      }else{
          ui->lb_ultra4->setText("0000");
      }
/**********************4#******************************/
      if(ui->ultra5->isChecked()){
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon5,Qt::WindingFill);
          }
         ui->lb_ultra5->setText(QString("%1").arg(dis[4]));
      }else{
          ui->lb_ultra5->setText("0000");
      }
/**********************5#******************************/
      if(ui->ultra6->isChecked()){
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon6,Qt::WindingFill);
          }
          ui->lb_ultra6->setText(QString("%1").arg(dis[5]));
      }else{
          ui->lb_ultra6->setText("0000");
      }
/**********************6#******************************/
      if(ui->ultra7->isChecked()){
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon7,Qt::WindingFill);
          }
          ui->lb_ultra7->setText(QString("%1").arg(dis[6]));
      }else{
          ui->lb_ultra7->setText("0000");
      }
/**********************7#******************************/
      if(ui->ultra8->isChecked()){
          if(ui->Ultra_Area->isChecked())
          {
             painter.drawPolygon(polygon8,Qt::WindingFill);
          }
          ui->lb_ultra8->setText(QString("%1").arg(dis[7]));
      }else{
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
    for(int i=0;i<8;i++)
    {
        painter.rotate(30);
        painter.drawLine(0,0,Mywidth/2 +10,0);
    }
    painter.rotate(60);
    painter.drawLine(0,0,Mywidth/2 +10,0);
    for(int i=0;i<2;i++)
    {
        painter.rotate(30);
        painter.drawLine(0,0,Mywidth/2 +10,0);
    }

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
        } else{
            st->setNum(i+90);
        }
        temp_x = cos((i)*CAMBER)*(Mywidth/2+10);
        temp_y = sin((i)*CAMBER)*(Mywidth/2+10);
        if(i<=210)
        {
            if((i<=90)&&(i>0))
            {
                temp_x += 10;
            }else{
                temp_x -= 10;
            }

            if(i==0)
            {
                temp_x +=5;
            }

            if(i==180)
            {
                temp_x -= 5;
            }else if(i==210){
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
         }else if(getModeSelect.checkedId()==Show_2M){
             if(distance[y]<=Set_First_Limit)
             {
                 painter.drawPoint(ShowPoint[y][0],ShowPoint[y][1]);
             }
         }else if(getModeSelect.checkedId()==Show_1M){
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

    polygon<<QPointF(0.0,0.0)
           <<QPointF(-(((Car_R+Safe_Distance)*Myhigh/2)/DIA)*tan(PI/12),
                     -((Safe_Distance+Car_R)*Myhigh/2)/DIA)
           <<QPointF((((Car_R+Safe_Distance)*Myhigh/2)/DIA)*tan(PI/12),
                     -((Safe_Distance+Car_R)*Myhigh/2)/DIA);

    painter.drawPolygon(polygon,Qt::WindingFill);
  //draw the car outline
    painter.setBrush(Qt::yellow);
    painter.drawEllipse(-((Car_R*Myhigh/2)/DIA),
                        -((Car_R*Myhigh/2)/DIA),
                        (2*Car_R*Myhigh/2)/DIA,
                        (2*Car_R*Myhigh/2)/DIA);
 //draw the laser outline
    painter.setBrush(Qt::black);
    painter.drawEllipse(-((Laser_R*Myhigh/2)/DIA),
                        -((Laser_R*Myhigh/2)/DIA),
                        (2*Laser_R*Myhigh/2)/DIA,
                        (2*Laser_R*Myhigh/2)/DIA);
 // display the state of the car
    painter.drawText(230,-Mywidth/2,"Normal  Run:");
    painter.drawText(230,-Mywidth/2+50,"Avoid Obstacle:");
    painter.drawText(230,-Mywidth/2+100,"Warning:");

    memset(ShowPoint,0,sizeof(ShowPoint));  //reset the result array
    painter.end();
    ui->label_main->setPixmap(*pixmap);
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
    if(!map.empty())
    {
        ui->pBtn_binarization->setChecked(false);
        img_binarization=false;
        ui->pBtn_binarization->setStyleSheet("background-color: rgb(167, 167, 125)");
    }

    using namespace cv;
    QString img_name = QFileDialog::getOpenFileName(this,tr("Open Image"),".",tr("Image Files(*.png *.jpg *.pgm *.bmp)"));

    QString info_name =img_name;
    info_name.replace(".pgm",".yaml");

    QFile infile(info_name);
    if(!infile.open(QIODevice::ReadOnly |QIODevice::Text))
    {
        qDebug()<<"can not open .yaml file!";
        return ;
    }
    QString info ;
    int cnt=0;
    while(!infile.atEnd())
    {
      info = infile.readLine();
      cnt++;
      if(cnt==2)
      {
          QString  res = info.left(info.indexOf("\n"));
          res =res.mid(res.indexOf(": ")+2);
          mapInfo.resolution =res.toFloat();
      }else if(cnt==3){
           QString res = info.left(info.indexOf("]"));
           res = res.mid(res.indexOf("[")+1);
           QString t =res.left(res.indexOf(","));
           mapInfo.word_x =t.toDouble();
           res =res.mid(res.indexOf(" ")+1);
           res =res.left(res.indexOf(","));
           mapInfo.word_y = res.toDouble();
           break;
      }
    }
    infile.close();

    map=imread(img_name.toLatin1().data());

    mapInfo.w=map.cols;
    mapInfo.h=map.rows;
    mapInfo.map_x = floor(abs(mapInfo.word_x/mapInfo.resolution) +0.5);
    mapInfo.map_y = mapInfo.h - floor(abs(mapInfo.word_y/mapInfo.resolution) +0.5);

    m_mapImage.GetQImage(map,text_img);

    ui->lbl_map->setPixmap(QPixmap::fromImage(text_img));
    ui->lbl_map->resize(text_img.width(),text_img.height());
    ui->lbl_map->setScaledContents(true);

    ui->lbl_map_info_res->setText(QString::number(mapInfo.resolution,'f',3));
    ui->lbl_map_info_size->setText(QString::number(mapInfo.w)+" x "+QString::number(mapInfo.h));

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

void MainWindow::reRefreshMap(void)
{
    if(!map.empty())
    {
        cv::Mat outmap;
        cv::Mat sample_map;
        m_mapImage.GetBinaryImage(map,th_binarization,outmap,sample_map);

        m_mapImage.GetQImage(outmap,text_img);  // outmap or sample_map?
    }
    redrawMap();
}

void MainWindow::redrawMap(void)
{
    if(!map.empty())
    {
        QImage img =text_img;
        //draw xy
        QPainter painter;
        painter.begin(&img);
        QPen pen(Qt::GlobalColor::red,3);
        painter.setPen(pen);
        QPoint o(mapInfo.map_x , mapInfo.map_y);
        painter.drawLine(o,QPoint(o.x()+zw::xyLength ,o.y()));
        pen.setColor(Qt::GlobalColor::green);
        painter.setPen(pen);
        painter.drawLine(o,QPoint(o.x(), o.y()-zw::xyLength));

        //draw car
        pen.setColor(Qt::GlobalColor::blue);
        painter.setPen(pen);
        carInfo.mx = floor(abs((carInfo.x -mapInfo.word_x)/mapInfo.resolution)+0.5);
        carInfo.my = mapInfo.h-floor(abs((carInfo.y-mapInfo.word_y)/mapInfo.resolution)+0.5);

        int x1 = carInfo.mx + floor(zw::carLength1*cos(carInfo.h)+0.5);
        int y1 = carInfo.my - floor(zw::carLength1*sin(carInfo.h)+0.5);
        int x2 = carInfo.mx + floor(zw::carLength1*cos(M_PI*5/6-carInfo.h)+0.5);
        int y2 = carInfo.my + floor(zw::carLength1*sin(M_PI*5/6-carInfo.h)+0.5);
        int x3 = carInfo.mx - floor(zw::carLength1*cos(carInfo.h-M_PI/6)+0.5);
        int y3 = carInfo.my + floor(zw::carLength1*sin(carInfo.h-M_PI/6)+0.5);

        QPolygon polygon;
        polygon <<QPoint(x1,y1)<<QPoint(x2,y2)<<QPoint(x3,y3);
        painter.drawPolygon(polygon,Qt::WindingFill);

        {
            pen.setColor(Qt::GlobalColor::darkGreen);
            painter.setPen(pen);
            for(int i=0;i<navi.g.vertexNum;i++)
            {
                QPoint c(floor(abs((navi.g.vertex[i].x -mapInfo.word_x)/mapInfo.resolution)+0.5),
                         mapInfo.h-floor(abs((navi.g.vertex[i].y-mapInfo.word_y)/mapInfo.resolution)+0.5));
                painter.drawEllipse(c,zw::vertex_radius,zw::vertex_radius);
                c.setX(c.rx()-zw::vertex_text_offset);
                c.setY(c.ry()+zw::vertex_text_offset);
                painter.drawText(c,QString::number(i));
            }
            QVector<qreal> dashes;
            qreal space=3;
            dashes <<5<<space<<5<<space;
            pen.setDashPattern(dashes);
            pen.setWidth(2);
            pen.setColor(Qt::GlobalColor::magenta);
            painter.setPen(pen);
            for(int i=0;i<navi.g.edgeNum;i++)
            {
                zw::CarPose vp =navi.g.vertex[navi.g.se[i].start];
                QPoint s (floor(abs((vp.x -mapInfo.word_x)/mapInfo.resolution)+0.5),
                              mapInfo.h-floor(abs((vp.y-mapInfo.word_y)/mapInfo.resolution)+0.5));
                vp =navi.g.vertex[navi.g.se[i].end];
                QPoint e (floor(abs((vp.x -mapInfo.word_x)/mapInfo.resolution)+0.5),
                          mapInfo.h-floor(abs((vp.y-mapInfo.word_y)/mapInfo.resolution)+0.5));
                painter.drawLine(s, e);
            }
        }

        {
            pen.setStyle(Qt::SolidLine);
            pen.setColor(Qt::GlobalColor::green);
            painter.setPen(pen);
            for( unsigned int i=1;i< nav_path.size();i++)
            {
                zw::CarPose vp= navi.g.vertex[nav_path[i-1]];
                QPoint s (floor(abs((vp.x -mapInfo.word_x)/mapInfo.resolution)+0.5),
                              mapInfo.h-floor(abs((vp.y-mapInfo.word_y)/mapInfo.resolution)+0.5));
                vp =navi.g.vertex[nav_path[i]];
                QPoint e (floor(abs((vp.x -mapInfo.word_x)/mapInfo.resolution)+0.5),
                          mapInfo.h-floor(abs((vp.y-mapInfo.word_y)/mapInfo.resolution)+0.5));
                painter.drawLine(s, e);
            }

        }

        painter.end();

        ui->lbl_map->setPixmap(QPixmap::fromImage(img));
        ui->lbl_map->resize(img.width(),img.height());
        ui->lbl_map->setScaledContents(true);
    }
}

void MainWindow::on_pBtn_binarization_clicked(bool checked)
{
    if(checked)
    {
        reRefreshMap();
        img_binarization=true;
        ui->pBtn_binarization->setStyleSheet("background-color: rgb(0, 255, 0);");
    }else{
        img_binarization=false;
        ui->pBtn_binarization->setStyleSheet("background-color: rgb(167, 167, 125)");
    }
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    if(img_binarization)
    {
        th_binarization=value;
        reRefreshMap();
    }
    QString str="二值化:";
    str += QString::number(th_binarization);
    ui->pBtn_binarization->setText(str);
    ui->horizontalSlider->setValue(th_binarization);

    m_mapImage.ShowBinaryImage("sample_map",map,th_binarization);
}

void MainWindow::on_Spin_Sample_Count_valueChanged(int arg1)
{
    m_mapImage.sample_cnt=arg1;
    m_mapImage.ShowBinaryImage("sample_map",map,th_binarization);
    reRefreshMap();
}

void MainWindow::on_Spin_Filter_Count_valueChanged(int arg1)
{
    m_mapImage.filter_cnt=arg1;
    m_mapImage.ShowBinaryImage("sample_map",map,th_binarization);
    reRefreshMap();
}

void MainWindow::PoseRefresh(void)
{
    getCarInfo();
    ui->lbl_robot_pose_x->setText(QString::number(carInfo.x,'f',3));
    ui->lbl_robot_pose_y->setText(QString::number(carInfo.y,'f',3));
    ui->lbl_robot_pose_h->setText(QString::number(carInfo.h,'f',3));
}

void MainWindow::getCarInfo(void)
{
    zw::Paras m_para;
    int32_t dat[3];
    zw::ParaGetSet  packInfo = {zw::R_REGISTER,3,(zw::ParaAddress)(zw::MSG_CONTROL+2),dat};
    m_para.GetAddressValue(packInfo);
    zw::Float2Int32 ff;
    ff.i=dat[0];
    carInfo.x =ff.f;
    ff.i=dat[1];
    carInfo.y =ff.f;
    ff.i=dat[2];
    carInfo.h =ff.f;
}

void MainWindow::on_pBtn_save_pose_clicked(bool checked)
{
    if(checked)
    {
        save_pose_file =true;
        ui->pBtn_save_pose->setStyleSheet("background-color: rgb(0, 255, 0);");
    }else{
        ui->pBtn_save_pose->setStyleSheet("background-color: rgb(167, 167, 125)");
        save_pose_file =false;
    }
}

void MainWindow::on_pBtn_clear_pose_clicked()
{
    clear_pose_file = true;
}

void MainWindow::SavePoseFiile(void)
{
     QString filePath="../pose.txt";
     static zw::CarInfo lastCarInfo={0,0,0,0,0};
     getCarInfo();
     QFile f(filePath);

     if(clear_pose_file)
     {
         clear_pose_file = false;
         if(f.exists())
         {
             f.remove();
         }
         lastCarInfo={0,0,0,0,0};
     }

     if( (abs((int)((carInfo.x -lastCarInfo.x)*100))>0) ||
         (abs((int)((carInfo.y -lastCarInfo.y)*100))>0) ||
         (abs((int)((carInfo.h -lastCarInfo.h)*20))>0) )
     {
         if(save_pose_file)
         {
             static int cnt=0;
             if(!f.exists())
                  cnt=0;

              if(!f.open(QIODevice::WriteOnly |QIODevice::Append |QIODevice::Text))
              {
                  qDebug()<<"Open pose file failed";
                  lastCarInfo = carInfo;
                  return ;
              }
              QTextStream txtOutput(&f);
              txtOutput<< cnt++
                       <<" "<<(int)(carInfo.x*1000)
                       <<" "<<(int)(carInfo.y*1000)
                       <<" "<<(int)(carInfo.h*1000)<<endl;
              f.close();
         }
         lastCarInfo = carInfo;
     }
}

void MainWindow::on_pBtn_start_navigation_clicked(bool checked)
{
    zw::Paras m_para;
    int32_t dat[3];
    zw::ParaGetSet  packInfo = {zw::W_REGISTER,3,(zw::ParaAddress)(zw::CONTROL+2),dat};
    if(checked)
    {
        zw::Float2Int32 ff;
        ff.f = ui->lbl_robot_epose_x->text().toFloat();
        dat[0] = ff.i;

        ff.f = ui->lbl_robot_epose_y->text().toFloat();
        dat[1] = ff.i;

        ff.f = ui->lbl_robot_epose_h->text().toFloat();
        dat[2] = ff.i;

        m_para.SetAddressValue(packInfo);
        m_tcpSocketClient->SendMsg(packInfo);

        packInfo={zw::R_REGISTER,1, zw::BTN_SWITCH,dat};
        m_para.GetAddressValue(packInfo);
        dat[0] |=KEY_START_NAV ;
        dat[0] |=KEY_NEW_GOAL ;
        ui->pBtn_start_navigation->setStyleSheet("background-color: rgb(0, 255, 0);");

    }else{
        packInfo={zw::R_REGISTER,1, zw::BTN_SWITCH,dat};
        m_para.GetAddressValue(packInfo);
        dat[0] &=(~KEY_START_NAV) ;
        dat[0] &=(~KEY_NEW_GOAL) ;
        ui->pBtn_start_navigation->setStyleSheet("background-color: rgb(167, 167, 125)");
    }
    packInfo.fuc =zw::W_REGISTER;
    m_para.SetAddressValue(packInfo);
    m_tcpSocketClient->SendMsg(packInfo);

    dat[0] &=(~KEY_NEW_GOAL) ;
    m_para.SetAddressValue(packInfo);
}

void MainWindow::on_pBtn_emergency_stop_clicked(bool checked)
{
    int32_t dat[1];
    zw::ParaGetSet packInfo={zw::R_REGISTER,1, zw::BTN_SWITCH,dat};
    zw::Paras m_para;
    m_para.GetAddressValue(packInfo);

    if(checked)
    {
        ui->pBtn_emergency_stop->setStyleSheet("background-color: rgb(0, 255, 0);");
        dat[0] |= KEY_EME_STOP;
    }else{
        ui->pBtn_emergency_stop->setStyleSheet("background-color: rgb(167, 167, 125)");
        dat[0] &= (~KEY_EME_STOP);
    }

    packInfo.fuc =zw::W_REGISTER;
    m_para.SetAddressValue(packInfo);

    m_tcpSocketClient->SendMsg(packInfo);

    m_tcpSocketClient->SendMsg(packInfo);
}

void MainWindow::on_pBtn_PID_confirm_clicked()
{
    zw::Paras m_para;
    int32_t dat[6];
    zw::ParaGetSet  packInfo = {zw::W_REGISTER,6,(zw::ParaAddress)(zw::ADD_PID+6),dat};
    dat[0] = ui->let_pose_pid_P->text().toInt();
    dat[1] = ui->let_pose_pid_I->text().toInt();
    dat[2] = ui->let_pose_pid_D->text().toInt();
    dat[3] = ui->let_angle_pid_P->text().toInt();
    dat[4] = ui->let_angle_pid_I->text().toInt();
    dat[5] = ui->let_angle_pid_D->text().toInt();
    m_para.SetAddressValue(packInfo);

    m_tcpSocketClient->SendMsg(packInfo);
}

void MainWindow::on_pBtn_nav_err_conf_clicked()
{
    int dat[2];
    navi.EndNodeDisErr =ui->lbl_robot_epose_err_x->text().toFloat();
    navi.EndNodeAngErr =ui->lbl_robot_epose_err_h->text().toFloat();
    dat[0]=(int)( navi.EndNodeDisErr*1000);
    dat[1]=(int)( navi.EndNodeAngErr*1000);
    zw::Paras m_para;
    zw::ParaGetSet  packInfo = {zw::W_REGISTER,2,zw::ADD_ERR,dat};
    m_para.SetAddressValue(packInfo);
    m_tcpSocketClient->SendMsg(packInfo);
}

void MainWindow::on_pBtn_open_inspection_file_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this,tr("Open inspection"),".",tr("txt Files(*.txt)"));
 //   QString filePath=ins_path;
    ins_path = filePath;
    QFile f(filePath);
    if(f.exists())
    {
         if(!f.open(QIODevice::ReadOnly |QIODevice::Text))
         {
             qDebug()<<"Open inspection file failed";
             return  ;
         }
         QTextStream in(&f);

         QString line=in.readLine() ;
         if(ins_Head != line)
         {
             qDebug()<<"Error inspection file";
             f.close();
             return;
         }
         QStringList list;
         line =in.readLine();
         list=line.split(QRegExp("\t"));

         navi.g.vertex.clear();
         navi.g.se.clear();

         navi.g.vertexNum = list.at(0).toInt();
         navi.g.edgeNum =list.at(1).toInt();
       //  navi.g.vertex.resize(navi.g.vertexNum);
       //  navi.g.se.resize(navi.g.edgeNum);

         line =in.readLine();
         for(int i=0;i<navi.g.vertexNum && (!line.isNull());i++)
         {
             list =line.split(QRegExp("\t"));
             zw::CarPose p;
             p.x = list.at(1).toInt()/1000.0;
             p.y = list.at(2).toInt()/1000.0;
             p.h = list.at(3).toInt()/1000.0;
            // navi.g.vertex[i]=(p);
             navi.g.vertex.push_back(p);
             line = in.readLine();
         }
         for(int i=0;i<navi.g.edgeNum && (!line.isNull());i++)
         {
             list =line.split(QRegExp("\t"));
             zw::Edge se;
             se.start =list.at(0).toInt();
             se.end =list.at(1).toInt();
           //  navi.g.se[i]=se;
             navi.g.se.push_back(se);

             line = in.readLine();
         }
         ui->lbl_vertex_number->setText("顶点:"+QString::number(navi.g.vertexNum));
         ui->lbl_edge_number->setText("边:"+QString::number(navi.g.edgeNum));
         navi.g.changed = true ;

         f.close();
    }else{
         qDebug()<<"inspection file  do not exist!";
    }
}

void MainWindow::on_pBtn_save_inspection_file_clicked()
{
    QString filePath=ins_path;
    QFile f(filePath);
    if(f.exists())
        f.remove();
    if(f.open(QIODevice::WriteOnly |QIODevice::Append |QIODevice::Text))
    {
          QTextStream txtOutput(&f);
          txtOutput<<ins_Head<<endl
                   <<navi.g.vertexNum<<"\t"<<navi.g.edgeNum<<endl;
          for(int i=0;i<navi.g.vertexNum;i++)
          {
              txtOutput<<i<<"\t"<<(int)(navi.g.vertex[i].x*1000)
                          <<"\t"<<(int)(navi.g.vertex[i].y*1000)
                          <<"\t"<<(int)(navi.g.vertex[i].h*1000)<<endl;
          }
          for(int i=0;i<navi.g.edgeNum;i++)
          {
              txtOutput<<navi.g.se[i].start<<"\t"<<navi.g.se[i].end<<endl;
          }
          f.close();
    }else{
         qDebug()<<"save inspection file failed";
    }
}

void MainWindow::on_pBtn_add_vertex_clicked()
{
    getCarInfo();
    zw::CarPose p{carInfo.x,carInfo.y,carInfo.h};
    navi.add_vertex(p);
    ui->lbl_vertex_number->setText("顶点:"+QString::number(navi.g.vertexNum));
}

void MainWindow::on_pBtn_delete_vertex_clicked()
{
    int index=ui->let_delete_vertex_index->text().toInt();
    navi.delete_vertex(index);
    ui->lbl_vertex_number->setText("顶点:"+QString::number(navi.g.vertexNum));
    ui->lbl_edge_number->setText("边:"+QString::number(navi.g.edgeNum));
}

void MainWindow::on_pBtn_add_edge_clicked()
{
    int start = ui->let_vertex_start_index->text().toInt();
    int end = ui->let_vertex_end_index->text().toInt();
    navi.add_edge(start, end);
    ui->lbl_edge_number->setText("边:"+QString::number(navi.g.edgeNum));
}

void MainWindow::on_pBtn_delete_edge_clicked()
{
    int start = ui->let_vertex_start_index->text().toInt();
    int end = ui->let_vertex_end_index->text().toInt();
    navi.delete_edge(start, end);
    ui->lbl_edge_number->setText("边:"+QString::number(navi.g.edgeNum));
}

static pthread_mutex_t g_tNaviMutex  = PTHREAD_MUTEX_INITIALIZER;

void MainWindow::on_pBtn_g_navi_clicked()
{
        int start = ui->let_vertex_start_index->text().toInt();
        int end = ui->let_vertex_end_index->text().toInt();
        std::vector<int> rpath ,path;
        nav_path.clear();
        if(navi.dijkstra(start, end, rpath))
        {
            for(int i=end;rpath[i]>=0;i=rpath[i])  //反向打印路径
                path.push_back(i);
            path.push_back(start);

            nav_path.resize(path.size());
            int i,j=0;
            for(i=path.size()-1;i>=0;i--)
            {
                nav_path[j++]=path[i];
            }
            qDebug()<<nav_path<<nav_path.size();
            nav_timer->stop();

            pthread_mutex_lock(&g_tNaviMutex );
            navi.firstIn = true;
            navi.k =0;
            navi.cmdCnt=0;
            pthread_mutex_unlock(&g_tNaviMutex );
            nav_timer->start();

        }else{
            qDebug()<<"can not reach !";
        }
        plot_g_navi = true;
}

void MainWindow::naviTimerupdate(void)
{
    zw::Paras m_para;
    int32_t dat[3];
    zw::ParaGetSet  packInfo = {zw::W_REGISTER,3,(zw::ParaAddress)(zw::CONTROL+2),dat};
    getCarInfo();
    zw::CarPose start={carInfo.x,carInfo.y,carInfo.h};
    zw::CarPose end;
    float errd ,errh;

    bool res=true;
    end = navi.g.vertex[nav_path[navi.k]];
    errd = sqrt((start.x-end.x)*(start.x-end.x) + (start.y-end.y)*(start.y-end.y));
    errh = fabs(zw::angle_diff(end.h,start.h));

    zw::Float2Int32 ff;
    ff.f = end.x;
    dat[0] = ff.i;
    ff.f = end.y;
    dat[1] = ff.i;
    ff.f = end.h;
    dat[2] = ff.i;

    pthread_mutex_lock(&g_tNaviMutex );
    if(navi.firstIn)
    {
       if(errd<zw::PassNodeErr){
          navi.cmdCnt =1;
          navi.firstIn =false;
       }else if(navi.cmdCnt == 0){
           m_para.SetAddressValue(packInfo);
           res=m_tcpSocketClient->SendMsg(packInfo);
           qDebug()<< "Send "<<nav_path[navi.k]<<";"<<"goal "<<end.x<<end.y<<end.h;

           navi.cmdCnt =1;

           packInfo={zw::R_REGISTER,1, zw::BTN_SWITCH,dat};
           m_para.GetAddressValue(packInfo);
           dat[0] |=KEY_START_NAV ;
           dat[0] |=KEY_NEW_GOAL ;
           packInfo.fuc =zw::W_REGISTER;
           m_para.SetAddressValue(packInfo);
           res=m_tcpSocketClient->SendMsg(packInfo);
           dat[0] &=(~KEY_START_NAV) ;
           dat[0] &=(~KEY_NEW_GOAL) ;
           m_para.SetAddressValue(packInfo);
       }
    }else{
        if(navi.cmdCnt==nav_path.size())
        {
            if((errd<= navi.EndNodeDisErr) && (errh<= navi.EndNodeAngErr)){
                nav_timer->stop();
                navi.firstIn =true;
                navi.k=0;
                navi.cmdCnt =0;
                qDebug()<<"Reach Goal !";
            }
        }else{
            if((errd<zw::PassNodeErr) && (navi.cmdCnt == navi.k+1))
            {
                navi.k++;
            }else if(navi.cmdCnt == navi.k){
                m_para.SetAddressValue(packInfo);
                res=m_tcpSocketClient->SendMsg(packInfo);
                qDebug()<< "Send "<<nav_path[navi.k]<<";"<<"goal "<<end.x<<end.y<<end.h;
                navi.cmdCnt ++;

                packInfo={zw::R_REGISTER,1, zw::BTN_SWITCH,dat};
                m_para.GetAddressValue(packInfo);
                dat[0] |=KEY_START_NAV ;
                dat[0] |=KEY_NEW_GOAL ;
                packInfo.fuc =zw::W_REGISTER;
                m_para.SetAddressValue(packInfo);
                res=m_tcpSocketClient->SendMsg(packInfo);
                dat[0] &=(~KEY_START_NAV) ;
                dat[0] &=(~KEY_NEW_GOAL) ;
                m_para.SetAddressValue(packInfo);
            }
        }
    }

    if(!res)
    {
        nav_timer->stop();
        navi.firstIn =true;
        navi.k=0;
        navi.cmdCnt =0;
        qDebug()<<"Can not send msg!";
    }
    pthread_mutex_unlock(&g_tNaviMutex );
}
