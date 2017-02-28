#include "../common/map_process.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include  <tf/transform_broadcaster.h>

void mapReceived(const nav_msgs::OccupancyGridConstPtr& grid);
void scanReceived(const sensor_msgs::LaserScanConstPtr& scan);


void getFirstPointCloud(sensor_msgs::PointCloud& pfcloud);

void sendtf(float x,float y ,float theta);

zw::MapProcess  m_mapProcess;

bool mapfinish=false;
bool firstFinish=false;

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

  bool mapflag=true;
  sensor_msgs::PointCloud pfcloud;
  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    ros::spinOnce();
    sendtf(0,0 ,0);

    if(mapfinish && mapflag)
    {
        mapflag =false;
        filter_map_pub.publish(m_mapProcess.filter_map);
    //    getFirstPointCloud(pfcloud);
     //   first_points_pub.publish(pfcloud);
    }
    if(firstFinish)
    {
        firstFinish =false;
        getFirstPointCloud(pfcloud);
        first_points_pub.publish(pfcloud);
    }
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}


void mapReceived(const nav_msgs::OccupancyGridConstPtr& grid)
{
    m_mapProcess.GetBinaryAndSample(grid,100,0,4);
    mapfinish=true;
}

void scanReceived(const sensor_msgs::LaserScanConstPtr& scan)
{
    if(mapfinish)
    {
      m_mapProcess.CalScan(scan,0.20);
      firstFinish =true;
    }
}

void getFirstPointCloud(sensor_msgs::PointCloud& pfcloud)
{

    pfcloud.header.frame_id="map";
    pfcloud.header.stamp=ros::Time::now();

    int num=0;
    for(int i=0;i<m_mapProcess.free_grid_Cell.size();i++)
    {
        if(m_mapProcess.free_grid_Cell[i].status==2)
            num++;
    }
  //  ROS_INFO("first optimize cell= %d/%d",num, (int)m_mapProcess.free_grid_Cell.size());

    pfcloud.points.resize(num);

    for(int i=0;i<num;i++)
    {
        pfcloud.points[i]= m_mapProcess.GetPoint(m_mapProcess.free_grid_Cell[i],
                                                 m_mapProcess.filter_map);
    }
}


void sendtf(float x,float y ,float theta)
{
    static tf::TransformBroadcaster br;
    tf::Transform tr;
    tr.setOrigin(tf::Vector3(x,y,0.0));
    tf::Quaternion q;
    q.setRPY(0,0,theta);
    tr.setRotation(q);
    br.sendTransform(tf::StampedTransform(tr,ros::Time::now(),"map","odom"));
}
