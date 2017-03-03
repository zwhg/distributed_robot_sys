#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


void amcl_PoseReceived(const geometry_msgs::PoseStamped pose);
void scan_PoseReceived(const geometry_msgs::PoseStamped pose);
//void google_PoseReceived(const a_robot_platform::SubmapList sublist);

bool PoseDiff(const geometry_msgs::PoseStamped & s,const geometry_msgs::PoseStamped & d);

#define PATH_NUM 3

nav_msgs::Path robot_path[PATH_NUM];
bool firstRec[PATH_NUM]={true};
float dis_diff=0.1;

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "main_path_publish");
    ROS_INFO("package_name:a_robot_platform  node_name:main_path_publish");
    ros::NodeHandle n;

    int pose_cnt;

    ros::NodeHandle nh("~");

    if(!nh.getParam("pose_cnt",pose_cnt))
        pose_cnt=1000;
    if(!nh.getParam("dis_diff",dis_diff))
        dis_diff=0.1;

    dis_diff *=dis_diff;

    for(int i=0;i<PATH_NUM;i++)
        robot_path[i].poses.clear();

    std::string pose_name[PATH_NUM]={
        "amcl_p",
        "scan_p",

        "submap_list"
    };
    std::string path_name[PATH_NUM]={
        "amcl_path",
        "scan_path",

        "submap_path"   //google mapping
    };

    ros::Subscriber pose_sub[PATH_NUM];
    pose_sub[0]=n.subscribe<geometry_msgs::PoseStamped>(pose_name[0], 1, amcl_PoseReceived);
    pose_sub[1]=n.subscribe<geometry_msgs::PoseStamped>(pose_name[1], 1, scan_PoseReceived);
  //  pose_sub[2]=n.subscribe<a_robot_platform::SubmapList>(pose_name[2], 1, google_PoseReceived);


    ros::Publisher path_pub[PATH_NUM];
    for(int i=0;i<PATH_NUM;i++)
       path_pub[i] = n.advertise<nav_msgs::Path>(path_name[i], 1, true);

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        for(int i=0;i<PATH_NUM;i++)
        {
            if(!firstRec[i])
            {
                 path_pub[i].publish(robot_path[i]);
            }
        }
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}

void  amcl_PoseReceived(const geometry_msgs::PoseStamped rpose)
{
    if(firstRec[0])
    {
        firstRec[0] =false;
        robot_path[0].header.stamp = rpose.header.stamp;
        robot_path[0].header.frame_id =rpose.header.frame_id;
        robot_path[0].poses.push_back(rpose);
    }
    else
    {
        int length =robot_path[0].poses.size();
        if(PoseDiff(robot_path[0].poses[length-1],rpose))
            robot_path[0].poses.push_back(rpose);
    }
}


void scan_PoseReceived(const geometry_msgs::PoseStamped rpose)
{
    if(firstRec[1])
    {
        firstRec[1] =false;
        robot_path[1].header.stamp = rpose.header.stamp;
        robot_path[1].header.frame_id =rpose.header.frame_id;
        robot_path[1].poses.push_back(rpose);
    }
    else
    {
        int length =robot_path[1].poses.size();
        if(PoseDiff(robot_path[1].poses[length-1],rpose))
            robot_path[1].poses.push_back(rpose);
    }
}



bool PoseDiff(const geometry_msgs::PoseStamped& s,const geometry_msgs::PoseStamped & d)
{
    float dx =s.pose.position.x -d.pose.position.x;
    float dy =s.pose.position.y -d.pose.position.y;

    return (dis_diff < (dx*dx+dy*dy));
}

