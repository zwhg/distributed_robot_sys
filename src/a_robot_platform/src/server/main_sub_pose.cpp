#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include "../common/modbus.h"


void PoseReceived(const geometry_msgs::PoseWithCovarianceConstPtr pose);

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "mian_sub_pose");
  ROS_INFO("package name:a_robot_platform  node name:main_sub_pose");

  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("amcl_pose", 2,PoseReceived);

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }

}


void PoseReceived(const geometry_msgs::PoseWithCovarianceConstPtr pose)
{
  int32_t ctr_msg[3];
  zw::ParaGetSet car_para={zw::R_REGISTER,3,zw::MSG_CONTROL+2,ctr_msg};
  zw::Float2Int32 mf;
  mf.f=(float)pose->pose.position.x;
  ctr_msg[0]=mf.i;
  mf.f=(float)pose->pose.position.y;
  ctr_msg[1]=mf.i;
  mf.f =(float)(tf::getYaw(pose->pose.orientation));
  ctr_msg[2]=mf.i;
  zw::Paras m_para;
  m_para.SetAddressValue(car_para);
}

