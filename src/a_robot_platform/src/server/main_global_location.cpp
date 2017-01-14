#include "../map/mapreadandwrite.h"


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_global_location");
  ROS_INFO("package_name:a_robot_platform  node_name:main_global_location");

  nav_msgs::OccupancyGrid grid;
  const std::string stem= "/home/zw/gt";
  zw::PgmAndYamlToOccupancy(grid,stem);
}
