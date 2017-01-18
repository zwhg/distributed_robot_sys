#include "amcl_location.h"

namespace zw {

std::vector<std::pair<int,int> > AmclNode::free_space_indices;

AmclNode::AmclNode():
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(NULL),
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
        laser_(NULL),
        private_nh_("~"),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
    map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
}

AmclNode::~AmclNode()
{

}

void AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{

}

}
