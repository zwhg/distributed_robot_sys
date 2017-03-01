#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

void PoseReceived(const geometry_msgs::PoseStamped pose);

bool PoseDiff(const geometry_msgs::PoseStamped & s,const geometry_msgs::PoseStamped & d);

nav_msgs::Path robot_path;
bool firstRec=true;
float dis_diff=0.1;

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "main_path_publish");
    ROS_INFO("package_name:a_robot_platform  node_name:main_path_publish");
    ros::NodeHandle n;

    std::string pose_topic;
    std::string path_topic;
    int pose_cnt;

    ros::NodeHandle nh("~");
    if(!nh.getParam("pose_topic",pose_topic))
        pose_topic="robot_pose";
    if(!nh.getParam("path_topic",path_topic))
        pose_topic="robot_path";
    if(!nh.getParam("pose_cnt",pose_cnt))
        pose_cnt=1000;
    if(!nh.getParam("dis_diff",dis_diff))
        dis_diff=0.1;

    dis_diff *=dis_diff;
    robot_path.poses.clear();

    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("robot_pose", 2,PoseReceived);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("robot_path", 1, true);
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        if(!firstRec)
        {
             path_pub.publish(robot_path);
        }
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}

void PoseReceived(const geometry_msgs::PoseStamped rpose)
{
    if(firstRec)
    {
        firstRec =false;
        robot_path.header.stamp = rpose.header.stamp;
        robot_path.header.frame_id =rpose.header.frame_id;
        robot_path.poses.push_back(rpose);
    }
    else
    {
        int length =robot_path.poses.size();
        if(PoseDiff(robot_path.poses[length-1],rpose))
            robot_path.poses.push_back(rpose);
    }
}


bool PoseDiff(const geometry_msgs::PoseStamped& s,const geometry_msgs::PoseStamped & d)
{
    float dx =s.pose.position.x -d.pose.position.x;
    float dy =s.pose.position.y -d.pose.position.y;

    return (dis_diff < (dx*dx+dy*dy));
}

