#include "../map/mapreadandwrite.h"

const std::string map_frame_id="map";

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_global_location");
  ROS_INFO("package_name:a_robot_platform  node_name:main_map_server");
  ros::NodeHandle n;
  nav_msgs::OccupancyGrid grid;
  const char * stem;
  if(argc>1)
    stem = argv[1];
  else
    stem= "/home/zw/gt";
  if(! zw::PgmAndYamlToOccupancy(grid,stem))
    return -1;
  grid.info.map_load_time=ros::Time::now();
  grid.header.frame_id=map_frame_id;
  grid.header.stamp=ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %3.2lf m/cell",grid.info.width,
           grid.info.height,grid.info.resolution);
  ros::Publisher map_pub;
  ros::Publisher metadata_pub;

  // Latched publisher for metadata
  metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  metadata_pub.publish(grid.info);

  // Latched publisher for data
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_pub.publish(grid);


  ros::spin();
  return 0;
}
