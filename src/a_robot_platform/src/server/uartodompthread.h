#ifndef UARTODOMPTHREAD_H
#define UARTODOMPTHREAD_H

#include <pthread.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <termios.h>
#include "../nav/nav.h"

namespace zw {


class UartOdomPthread
{

private:
  pthread_t id;
  static NavPara m_navPara;
  static geometry_msgs::Twist vel;

public:
  UartOdomPthread();
  ~UartOdomPthread();

private:
  static void cmd_keyCallback(const geometry_msgs::Twist::ConstPtr & cmd);
  static void timerCallback(const ros::TimerEvent &e);
  static void PoseReceived(const geometry_msgs::PoseStampedConstPtr pose);
  static void *MyPthread(void *temp);

  virtual	void *DoPthread(void);
};

}
#endif // UARTODOMPTHREAD_H
