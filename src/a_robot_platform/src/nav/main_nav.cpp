#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav.h"
#include "../common/map_process.h"


 static volatile zw::NavPara m_navPara={{0,0,0},{0,0,0},false,false};

void SubMapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
void PoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose);

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "main_nav");
    ROS_INFO("package_name:a_robot_platform  node_name:main_nav");

    ros::NodeHandle nh;

    ros::Subscriber submap_sub = nh.subscribe("subMap", 2,  SubMapReceived);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2,PoseReceived);

    ros::spin();
    ros::shutdown();
    return(0);
}

void PoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose)
{
     m_navPara.current.x =pose->pose.pose.position.x ;
     m_navPara.current.y =pose->pose.pose.position.y ;
}


void SubMapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
    float submap_fx = m_navPara.desired.x - m_navPara.current.x;
    float submap_fy = m_navPara.desired.y - m_navPara.current.y;

    float boundx = (msg->info.width-2)*msg->info.resolution*0.5 ;
    float boundy = (msg->info.height-2)*msg->info.resolution*0.5;
    float fk = (boundy/boundx) ;

    zw::CarPose tempGoal{submap_fx, submap_fy, m_navPara.desired.h};

    if( (fabs(submap_fx)>boundx) || (fabs(submap_fy)>boundy) )
    {
        if(submap_fx ==0)
        {
             tempGoal.x =0 ;
             tempGoal.y =((submap_fy>=0) ? boundy : -boundy);
        }else if(submap_fy ==0){
             tempGoal.x =((submap_fx>=0) ? boundx : -boundx);
             tempGoal.y =0;
        }else{
             float k = submap_fy /submap_fx ;
             if(fabs(k) <= fk)
             {
                 tempGoal.x = ((submap_fx >=0) ? boundx : -boundx);
                 tempGoal.y = k* tempGoal.x ;
             }else{
                 tempGoal.y = ((submap_fy >=0) ? boundy : -boundy);
                 tempGoal.x = tempGoal.y / k ;
             }
        }
    }
    int mi=floor(tempGoal.x / msg->info.resolution + 0.5) + msg->info.width / 2;
    int mj=floor(tempGoal.y / msg->info.resolution + 0.5) + msg->info.height /2;


    if(mi>=0 && mi<msg->info.width && mj>=0 && mj<msg->info.height)
    {
        std::vector<std::vector<char>> maze;
        maze.resize(msg->info.height ,std::vector<char>(msg->info.width,0));

        for(int i=0; i<msg->info.height; i++)
            for(int j=0; j<msg->info.width; j++)
            {
              maze[j][i] =  msg->data[j+i*msg->info.width];    //origin down-left
            }
        zw::Astart astar;
        astar.InitAstart(maze);
        int ox =msg->info.width/2 ;
        int oy =msg->info.height/2 ;

        zw::Point start(ox , oy);
        zw::Point end(mi,mj);

        ROS_INFO("start %d %d", start.x , start.y);
        ROS_INFO("end %d %d", end.x , end.y);

        if(start.x==end.x  && start.y==end.y)
            return ;
        if(maze[end.x][end.y]== zw::kOccGrid)
        {
            ROS_INFO("can not reach");
            return ;
        }

        std::list<zw::Point *> path =astar.GetPath(start, end, false);

        ROS_INFO("path %d",(int)path.size());
        for(auto &p:path)
        {
            ROS_INFO("%d ,%d",p->x,p->y);
        }
        ROS_INFO(" ");

    }else{
        ROS_ERROR("tempGoal out of bound");
    }
}
