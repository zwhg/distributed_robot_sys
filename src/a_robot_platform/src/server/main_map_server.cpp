#include "../map/mapreadandwrite.h"
#include "../common/map_process.h"

const std::string map_frame_id="map";

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_global_location");
  ROS_INFO("package_name:a_robot_platform  node_name:main_map_server");
  ros::NodeHandle n;
  nav_msgs::OccupancyGrid grid;
  nav_msgs::OccupancyGrid bmap;
  const char * stem;
  if(argc>1)
    stem = argv[1];
  else
    stem= "/home/zw/distributed_robot_sys/img/map/gtest1";
  if(! zw::PgmAndYamlToOccupancy(grid,stem))
    return -1;
  grid.info.map_load_time=ros::Time::now();
  grid.header.frame_id=map_frame_id;
  grid.header.stamp=ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %3.2lf m/cell",grid.info.width,
           grid.info.height,grid.info.resolution);
  ros::Publisher map_pub;
  ros::Publisher bmap_pub;

  // Latched publisher for data
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_pub.publish(grid);


  unsigned long dat_size=grid.info.width*grid.info.height;

  bmap.header=grid.header;
  bmap.info=grid.info;
  bmap.data.resize(dat_size,-1);

  char *m=new char[dat_size];
  zw:: map_process(m,grid);

  for (size_t k = 0; k < dat_size; ++k) {
      bmap.data[k]=m[k];
  }

  delete m;

  bmap_pub = n.advertise<nav_msgs::OccupancyGrid>("bmap", 1, true);
  bmap_pub.publish(bmap);

  ros::spin();
  ros::shutdown();

  return 0;
}
