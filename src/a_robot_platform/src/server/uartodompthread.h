#ifndef UARTODOMPTHREAD_H
#define UARTODOMPTHREAD_H

#include <pthread.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <termios.h>

namespace zw {


#define Gyro_Gr	  0.0005326f
#define Acc_Mss  ((8.0*9.81)/65536.0)


class UartOdomPthread
{

private:
  pthread_t id;

public:
  UartOdomPthread();
  ~UartOdomPthread();

private:
  static void timerCallback(const ros::TimerEvent &e);
  static void *MyPthread(void *temp);

  virtual	void *DoPthread(void);
};

}


#endif // UARTODOMPTHREAD_H
