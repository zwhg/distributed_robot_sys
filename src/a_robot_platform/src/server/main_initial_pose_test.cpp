#include "../common/map_process.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include  <tf/transform_broadcaster.h>
#include <fcntl.h>

void mapReceived(const nav_msgs::OccupancyGridConstPtr& grid);
void scanReceived(const sensor_msgs::LaserScanConstPtr& scan);

void getOptimizePointCloud(sensor_msgs::PointCloud pfcloud[]);
void sendtf(float x,float y ,float theta);


zw::MapProcess  m_mapProcess;

bool mapfinish=false;
bool firstFinish=false;



int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_initial_pose_test");
  ROS_INFO("package_name:a_robot_platform  node_name:main_initial_pose_test");
#if 0
  float a[3]={1.235,2.356,265.25};

  static int i=0;
  const char * filePath="../test.txt";
  if(i==0)
  {
      if(access(filePath,F_OK)==0)
      {
          remove(filePath);
          printf("delete ok !\n");
      }
  }
  for(;i<100;i++)
  {
      std::string str=std::to_string(i)+" "+
                      std::to_string((int)(a[0]*1000))+" "+
                      std::to_string((int)(a[1]*1000))+" "+
                      std::to_string((int)(a[2]*1000))+" "+
                      std::to_string((int)(a[0]*1000))+" "+
                      std::to_string((int)(a[1]*1000))+" "+
                      std::to_string((int)(a[2]*1000))+"\n";
      int len =str.length();
      const char * buf = str.c_str();

      int fd=open(filePath,O_CREAT|O_WRONLY|O_APPEND,S_IRWXU|S_IRWXG|S_IRWXO);
      if(-1==fd)
      {
        printf("test.txt not exist !\n");
      }
      write(fd,buf,len);
      close(fd);
  }
#endif
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  if(!nh.getParam("optimize1",m_mapProcess.optimize[0]))
      m_mapProcess.optimize[0]=0.2;
  if(!nh.getParam("optimize2",m_mapProcess.optimize[1]))
      m_mapProcess.optimize[1]=0.1;
  if(!nh.getParam("optimize3",m_mapProcess.optimize[2]))
      m_mapProcess.optimize[2]=0.5;
  if(!nh.getParam("optimize4",m_mapProcess.optimize[3]))
      m_mapProcess.optimize[3]=0.5;
  if(!nh.getParam("optimize5",m_mapProcess.optimize[4]))
      m_mapProcess.optimize[4]=0.5;
  if(!nh.getParam("laser_skip",m_mapProcess.laser_skip))
      m_mapProcess.laser_skip =20;
  if(!nh.getParam("filter_cnt",m_mapProcess.filter_cnt))
      m_mapProcess.laser_skip =4;

  ROS_INFO("optimize=[%6.2f %6.2f %6.2f]",m_mapProcess.optimize[0],
          m_mapProcess.optimize[1],m_mapProcess.optimize[2]);

  ros::Subscriber map_sub = n.subscribe("bmap", 1, mapReceived);
  ros::Subscriber scan_sub =n.subscribe("scan",1, scanReceived);

  ros::Publisher filter_map_pub;
  ros::Publisher optimize_points_pub[OPTIMIZE];
  filter_map_pub =  n.advertise<nav_msgs::OccupancyGrid>("filter_map", 1, true);
  for(int i=0;i<OPTIMIZE;i++)
  {
    std::string name="optimize_"+std::to_string(i+1);
    optimize_points_pub[i] =n.advertise<sensor_msgs::PointCloud>(name,1, true);
  }


  bool mapflag=true;
  sensor_msgs::PointCloud pfcloud[OPTIMIZE];
  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    ros::spinOnce();
    sendtf(0,0 ,0);

    if(mapfinish && mapflag)
    {
        mapflag =false;
        filter_map_pub.publish(m_mapProcess.filter_map);
    }
    if(firstFinish)
    {
        firstFinish =false;
        getOptimizePointCloud(pfcloud);
        for(int i=0;i<OPTIMIZE;i++)
          optimize_points_pub[i].publish(pfcloud[i]);
    }
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}


void mapReceived(const nav_msgs::OccupancyGridConstPtr& grid)
{
    m_mapProcess.GetBinaryAndSample(grid,100,0,m_mapProcess.filter_cnt);
    mapfinish=true;
}

void scanReceived(const sensor_msgs::LaserScanConstPtr& scan)
{
    if(mapfinish)
    {
      m_mapProcess.CalScan(scan);
      firstFinish =true;
    }
}

void getOptimizePointCloud(sensor_msgs::PointCloud pfcloud[])
{
   int num[OPTIMIZE]={0};

   for(int i=0;i<m_mapProcess.free_grid_Cell.size();i++)
   {
       switch (m_mapProcess.free_grid_Cell[i].status) {
       case 5:{
           num[4]++;
       }
       case 4:{
           num[3]++;
       }
       case 3:{
           num[2]++;
       }
       case 2:{
           num[1]++;
       }
       case 1:{
           num[0]++;
       }
       default:
           break;
       }
   }
   //ROS_INFO("optimize %d/%d",num[1], num[0]);

   for(int i=0;i<OPTIMIZE;i++)
   {
       pfcloud[i].header.frame_id="map";
       pfcloud[i].header.stamp=ros::Time::now();
       pfcloud[i].points.resize(num[i]);
       for(int j=0;j<num[i];j++)
           pfcloud[i].points[j]=m_mapProcess.GetPoint(m_mapProcess.free_grid_Cell[j],
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


