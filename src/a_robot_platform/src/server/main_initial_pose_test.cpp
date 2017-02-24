#include "../common/map_process.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

void mapReceived(const nav_msgs::OccupancyGridConstPtr& grid);
void scanReceived(const sensor_msgs::LaserScanConstPtr& scan);

zw::MapProcess  m_mapProcess;

int mapfinish=0;

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_initial_pose_test");
  ROS_INFO("package_name:a_robot_platform  node_name:main_initial_pose_test");

  ros::NodeHandle n;
  ros::Subscriber map_sub = n.subscribe("map", 1, mapReceived);
  ros::Subscriber scan_sub =n.subscribe("scan",1, scanReceived);

  ros::Publisher filter_map_pub;
  ros::Publisher first_points_pub;
  filter_map_pub =  n.advertise<nav_msgs::OccupancyGrid>("filter_map", 1, true);
  first_points_pub =n.advertise<sensor_msgs::PointCloud>("first_cloud",1, true);

  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    ros::spinOnce();

    if(mapfinish==1)
    {
        filter_map_pub.publish(m_mapProcess.filter_map);
    }
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}


void mapReceived(const nav_msgs::OccupancyGridConstPtr& grid)
{
    m_mapProcess.GetBinaryAndSample(grid,100,0,4);
    mapfinish=1;
}

void scanReceived(const sensor_msgs::LaserScanConstPtr& scan)
{
    if(mapfinish==1)
      m_mapProcess.CalScan(scan,0.4);
}
