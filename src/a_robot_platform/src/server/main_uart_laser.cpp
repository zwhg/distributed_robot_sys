#include <ros/ros.h>
#include <iostream>
#include <pthread.h>
#include <qdebug.h>
#include "../common/common.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/LaserScan.h"
#include "uartlaser.h"
#include "udpsocketserver.h"
#include <QtNetwork>

static int32_t startOrStop = 1;	// 1是start，0是stop
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static void publish_scan(ros::Publisher *pub, double *dist, int count, ros::Time start, double scan_time);
static void startStopCB(const std_msgs::Int32::ConstPtr msg);

pthread_t id;
void *send_ultrasonic(void*);
char msg[] = "UDP process is successful,here is main_uart_laser";

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_uart_laser");
  ROS_INFO("package_name:a_robot_platform  node_name:main_uart_laser");
  qDebug()<<"qDebug ok!";

  ros::NodeHandle n;
  zw::UartLaser m_lsRadar;
  int32_t ret = -1;
  const char * addrPort;
  if(argc==1)
      addrPort="/dev/ttyUSB0";
  else
      addrPort =argv[1];

  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
  ros::Subscriber stop_sub = n.subscribe<std_msgs::Int32>("main_uart_laser/startOrStop", 10, startStopCB);
  ret = m_lsRadar.OpenSerial(addrPort, B230400);
  if(ret<0)
      return -1;
  m_lsRadar.StartScan();
  bool isStarted = true;

  double angle[PACKLEN + 10];
  double distance[PACKLEN + 10];
  double data[PACKLEN + 10];
  double speed;

  ros::Time starts = ros::Time::now();
  ros::Time ends = ros::Time::now();
 // pthread_create(&id,NULL,send_ultrasonic,(void*)msg);

  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    ros::spinOnce();
   // zw::UdpSocketServer m_udpserver;

    pthread_mutex_lock(&mutex);
    if(isStarted && 0 == startOrStop)              // 当前正在扫描且要求停止
    {
      ROS_INFO("stop");
      m_lsRadar.StopScan();
      isStarted = false;
    }
    else if(!isStarted && 1 == startOrStop)	   // 当前未扫描且要求开始扫描
    {
      ROS_INFO("start");
      m_lsRadar.StartScan();
      isStarted = true;
    }
    pthread_mutex_unlock(&mutex);
    if(!isStarted)
      continue;

    memset(data, 0, sizeof(data));
    ret = m_lsRadar.GetScanData(angle, distance, PACKLEN, &speed);
    ends = ros::Time::now();
    float scan_duration = (ends - starts).toSec() * 1e-3;
    publish_scan(&scan_pub, distance, ret, starts, scan_duration);
    loop_rate.sleep();
    starts = ends;
  }

  m_lsRadar.StopScan();
  m_lsRadar.CloseSerial();
  ros::shutdown();
  //pthread_detach(id);
  return 0;
}

void *send_ultrasonic(void *arg)
{
    //  qDebug()<<arg;
      while(1)
      {
       //  zw::UdpSocketServer m_udpserver;
        ROS_INFO("this is laser constructor");
        double laser_dis[PACKLEN];
        zw:: Paras::get_distance(laser_dis);
        QUdpSocket *Udp_Sender = new QUdpSocket();
        QByteArray datagram;
        zw::floatTobyte  FtB;
        for(int i=0;i<720;i++)
        {
             FtB.ff =  laser_dis[i];
             for(int j=0;j<4;j++)
             {
                 datagram.append(FtB.fb[j]);
             }
              Udp_Sender->writeDatagram(datagram.data(),datagram.size(),QHostAddress::Broadcast,45454);
              datagram.clear();
        }
         sleep(1);
      }
}

void publish_scan(ros::Publisher *pub, double *dist, int32_t count, ros::Time start, double scan_time)
{
  static int32_t scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.stamp = start;
  scan_msg.header.frame_id = "laser_link";
  scan_count++;

  scan_msg.angle_min = 3.1415926;
  scan_msg.angle_max = -3.1415926;
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(count - 1);
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (double)(count - 1);

  scan_msg.range_min = zw:: kMinLaserRange;
  scan_msg.range_max = zw:: kMaxLaserRange;
  scan_msg.intensities.resize(count);
  scan_msg.ranges.resize(count);


  for (int i = 0; i < count; i++)
  {
    if (dist[i] == 0.0)
      scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
    else
      scan_msg.ranges[i] = dist[i] / 1000.0;
    scan_msg.intensities[i] = 0;
  }
  pub->publish(scan_msg);
}

void startStopCB(const std_msgs::Int32::ConstPtr msg)
{
  pthread_mutex_lock(&mutex);
  startOrStop = msg->data;
  pthread_mutex_unlock(&mutex);
}
