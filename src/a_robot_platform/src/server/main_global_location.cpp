#include <signal.h>
#include "../location/amcl_location.h"


boost::shared_ptr<zw::AmclNode> amcl_node_ptr;

void sigintHandler(int sig)
{
  // Save latest pose as we're shutting down.
 // amcl_node_ptr->savePoseToServer();
  ros::shutdown();
}


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "main_global_location");
  ROS_INFO("package_name:a_robot_platform  node_name:main_global_location");

  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  amcl_node_ptr.reset(new zw::AmclNode());

  if (argc == 1)
  {
    // run using ROS input
    ros::spin();
  }
  else if ((argc == 3) && (std::string(argv[1]) == "--run-from-bag"))
  {
    amcl_node_ptr->runFromBag(argv[2]);
  }

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}

