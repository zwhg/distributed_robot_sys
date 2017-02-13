#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
//#include <qdebug.h>

void mapCallback(nav_msgs::OccupancyGrid::ConstPtr& grid)
{
//   ROS_INFO("mapCallback receive data:",grid.info);
      ROS_INFO("mapCallback receive data:",grid);
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"main_pathplan_server");
  ROS_INFO("package_name: a_robot_platform  node_name: main_pathplan_server");
//qDebug()<<"qDebug ok!";
  ros::NodeHandle n;
  ros::Subscriber map_sub = n.subscribe("map",1,mapCallback);
  ros::spin();
  return 0;
}
