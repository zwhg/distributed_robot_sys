#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "uartlaser.h"
geometry_msgs::Twist msg;
void MoveMode(double linear_speed,double angular_speed)
{
      msg.linear.x = linear_speed;             //range  0~0.5
      msg.angular.z = angular_speed;           //range  0~1
}

int main(int argc, char **argv)
{
  // Set up ROS.//
  ros::init(argc, argv, "main_test_v");
  ROS_INFO("package_name:a_robot_platform  node_name:main_test_v");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  ros::Rate loop_rate(300);
  int count = 0;
  while(ros::ok())
  {
     //ROS_INFO("main_test_v");
     //ROS_INFO("count:%d\n",count);
//      if(count<=5850)  //左转180度   90度 (2755,0.5)
//         MoveMode(0.0,0.5);
//      else if(count<=5950)
//         MoveMode(0.0,0.0);
//      else if(count<=11650)  //右转180度
//         MoveMode(0.0,-0.5);
//      else
//         MoveMode(0.0,0.0);
/*****************************************************************/
/*   if(distance[180]>=1000)
	MoveMode(0.3,0);
   else if(distance[180]>=500)
	MoveMode(0.2,0);
   else if(distance[180]>=100)
        MoveMode(0.1,0);

       if(count<=1200)
          MoveMode(0.2,0); //直行
       else if(count<=1300)
          MoveMode(0.0,0.0);//停止
       else if(count<=7250)
          MoveMode(0.0,0.5);
       else if(count<=7350)
          MoveMode(0.0,0.0);
       else if(count<=8550)
          MoveMode(0.2,0.0);
       else
          MoveMode(0.0,0.0);
      vel_pub.publish(msg);
      loop_rate.sleep();
      count++;*/
  }
}
