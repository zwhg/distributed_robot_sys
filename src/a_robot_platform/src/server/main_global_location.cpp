#include "../map/mapreadandwrite.h"

void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_global_location");
  ROS_INFO("package_name:a_robot_platform  node_name:main_global_location");
  ros::NodeHandle n;
  ros::Subscriber map_sub=n.subscribe("map",1,mapReceived);

}

void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{

}
