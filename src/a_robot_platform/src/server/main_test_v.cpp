#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
geometry_msgs::Twist msg;
void MoveMode(double linear_speed,double angular_speed)
{
      msg.linear.x = linear_speed;   //range  0~0.5
      msg.angular.z = angular_speed;  //range  0~1
}
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_test_v");
  ROS_INFO("package_name:a_robot_platform  node_name:main_test_v");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  ros::Rate loop_rate(10);
//  int count = 0;
  while(ros::ok())
  {
      MoveMode(0.1,0.0);
      vel_pub.publish(msg);
      loop_rate.sleep();
  // count++;
  }
}
