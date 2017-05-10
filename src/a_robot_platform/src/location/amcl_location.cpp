#include "amcl_location.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "../map/probability_values.h"
#include "../common/map_process.h"

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
        scan_processor(),
        add_close_loop(true),
        use_amcl_pose(false)
{
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);
    paraInit();
    cloud_pub_interval.fromSec(1.0);
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new TransformListenerWrapper();

    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
    particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);

    global_loc_srv_ = nh_.advertiseService("global_localization",
                                               &AmclNode::globalLocalizationCallback,this);
    nomotion_update_srv_= nh_.advertiseService("request_nomotion_update",
                                               &AmclNode::nomotionUpdateCallback, this);
    set_map_srv_= nh_.advertiseService("set_map", &AmclNode::setMapCallback, this);

    laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
    laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_,
                                                            odom_frame_id_, 100);
    laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,this, _1));
    initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);

    test =nh_.advertise<sensor_msgs::PointCloud>("test",1, true);


    pose_pub_amcl =nh_.advertise<geometry_msgs::PoseStamped>("amcl_p",2,true);
    pose_pub_scan =nh_.advertise<geometry_msgs::PoseStamped>("scan_p",2,true);

    subMap_pub =nh_.advertise<nav_msgs::OccupancyGrid>("subMap", 1, true);

    if(use_map_topic_) {
      map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
      ROS_INFO("Subscribed to map topic.");
    } else {
      requestMap();
    }
    m_force_update = false;

    // 15s timer to warn on lack of receipt of laser scans, #5209
    laser_check_interval_ = ros::Duration(15.0);
    check_laser_timer_ = nh_.createTimer(laser_check_interval_,
                                         boost::bind(&AmclNode::checkLaserReceived, this, _1));
}

AmclNode::~AmclNode()
{
    freeMapDependentMemory();
    delete laser_scan_filter_;
    delete laser_scan_sub_;
    delete tfb_;
    delete tf_;
}

void AmclNode::paraInit()
{
    //When set to true, AMCL will subscribe to the map topic rather than
    //making a service call to receive its map
    if(!private_nh_.getParam("use_map_topic",use_map_topic_))
        use_map_topic_ =true;
    //When set to true, AMCL will only use the first map it subscribes to,
    //rather than updating each time a new one is received.
    if(!private_nh_.getParam("first_map_only",first_map_only_))
        first_map_only_ =true;

    //Maximum rate (Hz) at which scans and paths are published for visualization,
    //-1.0 to disable.
    double tmp;
    if(!private_nh_.getParam("gui_publish_rate",tmp))
        tmp = -1.0;
    gui_publish_period = ros::Duration(1.0/tmp);
    //Maximum rate (Hz) at which to store the last estimated pose and covariance
    //to the parameter server, in the variables ~initial_pose_* and ~initial_cov_*.
    //This saved pose will be used on subsequent runs to initialize the filter.
    //-1.0 to disable.
    if(!private_nh_.getParam("save_pose_rate",tmp))
        tmp =0.5;
    save_pose_period = ros::Duration(1.0/tmp);

    //Minimum scan range to be considered; -1.0 will cause the laser's reported
    //minimum range to be used.
    if(!private_nh_.getParam("laser_min_range",laser_min_range_))
        laser_min_range_ =-1.0;
    //Maximum scan range to be considered; -1.0 will cause the laser's reported
    //maximum range to be used.
    if(!private_nh_.getParam("laser_max_range",laser_max_range_))
        laser_max_range_ =-1.0;
    //How many evenly-spaced beams in each scan to be used when updating the filter.
    if(!private_nh_.getParam("laser_max_beams",max_beams_))
        max_beams_ =30;

    // ROS_INFO("use_map_topic=%d",use_map_topic_);
    // Minimum allowed number of particles.
    if(!private_nh_.getParam("min_particles",min_particles_))
        min_particles_ =500;
    //Mamimum allowed number of particles.
    if(!private_nh_.getParam("max_particles",max_particles_))
        max_particles_ =5000;

    //Maximum error between the true distribution and the estimated distribution.
    if(!private_nh_.getParam("kld_err",pf_err_))
        pf_err_ =0.01;
    //Upper standard normal quantile for (1 - p), where p is the probability that
    //the error on the estimated distrubition will be less than kld_err
    if(!private_nh_.getParam("kld_z",pf_z_))
        pf_z_ =0.99;

    //Specifies the expected noise in odometry's rotation estimate from the rotational
    //component of the robot's motion.
    if(!private_nh_.getParam("odom_alpha1",alpha1_))
        alpha1_ =0.2;
    //Specifies the expected noise in odometry's rotation estimate from the translational
    //component of the robot's motion.
    if(!private_nh_.getParam("odom_alpha2",alpha2_))
        alpha2_ =0.2;
    //Specifies the expected noise in odometry's translation estimate from the translational
    //component of the robot's motion.
    if(!private_nh_.getParam("odom_alpha3",alpha3_))
        alpha3_ =0.2;
    //Specifies the expected noise in odometry's translation  estimate from the rotational
    //component of the robot's motion.
    if(!private_nh_.getParam("odom_alpha4",alpha4_))
        alpha4_ =0.2;
    //Translation-related noise parameter (only used if model is omni)
    if(!private_nh_.getParam("odom_alpha5",alpha5_))
        alpha5_ =0.2;

    //When true skips laser scans when a scan doesn't work for a
    //majority of particles
    if(!private_nh_.getParam("do_beamskip",do_beamskip_))
        do_beamskip_ =false;
    //Distance from a valid map point before scan is considered invalid
    if(!private_nh_.getParam("beam_skip_distance",beam_skip_distance_))
        beam_skip_distance_ =0.5;
    //Ratio of samples for which the scans are valid to consider as valid scan
    if(!private_nh_.getParam("beam_skip_threshold",beam_skip_threshold_))
        beam_skip_threshold_ =0.3;
    //
    if(!private_nh_.getParam("beam_skip_error_threshold_",beam_skip_error_threshold_))
        beam_skip_error_threshold_ =0.9;

    //Mixture weight for the z_hit part of the model.
    if(!private_nh_.getParam("laser_z_hit",z_hit_))
        z_hit_ =0.95;
    //Mixture weight for the z_short part of the model.
    if(!private_nh_.getParam("laser_z_short",z_short_))
        z_short_ =0.1;
    //Mixture weight for the z_max part of the model.
    if(!private_nh_.getParam("laser_z_max",z_max_))
        z_max_ =0.05;
    //Mixture weight for the z_rand part of the model
    if(!private_nh_.getParam("laser_z_rand",z_rand_))
        z_rand_ =0.05;
    //Standard deviation for Gaussian model used in z_hit part of the model.
    if(!private_nh_.getParam("laser_sigma_hit",sigma_hit_))
        sigma_hit_ =0.2;
    //Exponential decay parameter for z_short part of model.
    if(!private_nh_.getParam("laser_lambda_short",lambda_short_))
        lambda_short_ =0.1;
    //Maximum distance to do obstacle inflation on map,
    //for use in likelihood_field model.
    if(!private_nh_.getParam("laser_likelihood_max_dist",laser_likelihood_max_dist_))
        laser_likelihood_max_dist_ =2.0;
    // laser model
    std::string tmp_model_type;
    if(!private_nh_.getParam("laser_model_type",tmp_model_type))
        tmp_model_type =std::string("likelihood_field");
    if(tmp_model_type == "beam")
      laser_model_type_ = LASER_MODEL_BEAM;
    else if(tmp_model_type == "likelihood_field")
      laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
    else if(tmp_model_type == "likelihood_field_prob")
      laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
    else
      laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;

    //Odometry Model Parameters
    if(!private_nh_.getParam("odom_model_type",tmp_model_type))
        tmp_model_type=std::string("diff");
    if(tmp_model_type == "diff")
        odom_model_type_ = ODOM_MODEL_DIFF;
    else if(tmp_model_type == "omni")
        odom_model_type_ = ODOM_MODEL_OMNI;
    else if(tmp_model_type == "diff-corrected")
        odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
    else if(tmp_model_type == "omni-corrected")
        odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
    else
        odom_model_type_ = ODOM_MODEL_DIFF;


    //Translational movement required before performing a filter update.
    if(!private_nh_.getParam("update_min_d",d_thresh_))
        d_thresh_ =0.2;
    //Rotational movement required before performing a filter update.
    if(!private_nh_.getParam("update_min_a",a_thresh_))
        a_thresh_ =M_PI/6.0;
    //Number of filter updates required before resampling.
    if(!private_nh_.getParam("resample_interval",resample_interval_))
        resample_interval_ =2;
    //Time with which to post-date the transform that is published,
    //to indicate that this transform is valid into the future.
    double tmp_tol;
    if(!private_nh_.getParam("transform_tolerance",tmp_tol))
        tmp_tol=0.1;
    transform_tolerance_.fromSec(tmp_tol);
    //Exponential decay rate for the slow average weight filter,
    //used in deciding when to recover by adding random poses.
    //A good value might be 0.001.
    if(!private_nh_.getParam("recovery_alpha_slow",alpha_slow_))
        alpha_slow_ =0.001;
    //Exponential decay rate for the fast average weight filter,
    //used in deciding when to recover by adding random poses.
    //A good value might be 0.1.
    if(!private_nh_.getParam("recovery_alpha_fast",alpha_fast_))
        alpha_fast_ =0.1;

    //When true (the default), publish results via TF.  When false, do not.
    if(!private_nh_.getParam("tf_broadcast",tf_broadcast_))
        tf_broadcast_ =true;
    //Which frame to use for odometry.
    if(!private_nh_.getParam("odom_frame_id",odom_frame_id_))
        odom_frame_id_ =std::string("odom");
    //Which frame to use for the robot base.
    if(!private_nh_.getParam("base_frame_id",base_frame_id_))
        base_frame_id_ =std::string("base_link");
    //The name of the coordinate frame published by the localization system.
    if(!private_nh_.getParam("global_frame_id",global_frame_id_))
        global_frame_id_ =std::string("map");

    {
       double bag_scan_period;
       if(!private_nh_.getParam("bag_scan_period", bag_scan_period))
           bag_scan_period=-1.0;
        bag_scan_period_.fromSec(bag_scan_period);
    }

    if(!private_nh_.getParam("pose_diff",scan_processor.poseDiff))
        scan_processor.poseDiff=0.01;

    if(!private_nh_.getParam("angle_diff",scan_processor.angleDiff))
        scan_processor.angleDiff=0.1;

    if(!private_nh_.getParam("map_depth_num",scan_processor.numDepth))
        scan_processor.numDepth=2;

    if(!private_nh_.getParam("publish_scan",scan_processor.publishScan))
        scan_processor.publishScan=false;

    if(!private_nh_.getParam("write_pose",scan_processor.writePose))
        scan_processor.writePose=false;

    if(!private_nh_.getParam("add_close_loop",add_close_loop))
        add_close_loop=true;

    if(!private_nh_.getParam("use_amcl_pose",use_amcl_pose))
        use_amcl_pose=false;

    float submap_res;;
    int submap_width,submap_height;;
    if(!private_nh_.getParam("subMap_resolution",submap_res))
        submap_res=0.1;
    if(!private_nh_.getParam("subMap_width",submap_width))
        submap_width=60;
    if(!private_nh_.getParam("subMap_height",submap_height))
        submap_height=60;

   scan_processor.subMap.info.resolution =submap_res;
   scan_processor.subMap.info.width =submap_width;
   scan_processor.subMap.info.height =submap_height;

   updatePoseFromServer();
}

void AmclNode::updatePoseFromServer()
{
    //initial pose mean and covariance,used to initialize filter with Gaussion distribution
    // Check for NAN on input from param server, #5239
    if(!private_nh_.getParam("initial_pose_x",init_pose_[0]))
       init_pose_[0]= 0.0;
    if(!private_nh_.getParam("initial_pose_y",init_pose_[1]))
       init_pose_[1]= 0.0;
    if(!private_nh_.getParam("initial_pose_a",init_pose_[2]))
       init_pose_[2]= 0.0;
    if(!private_nh_.getParam("initial_cov_xx",init_cov_[0]))
       init_cov_[0]= 0.5 * 0.5;
    if(!private_nh_.getParam("initial_cov_yy",init_cov_[1]))
       init_cov_[1]= 0.5 * 0.5;
    if(!private_nh_.getParam("initial_cov_aa",init_cov_[2]))
       init_cov_[2]= (M_PI/12.0) * (M_PI/12.0);
}

void AmclNode::checkLaserReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
  if(d > laser_check_interval_)
  {
    ROS_WARN("No laser scan received (and thus no pose updates have been published) "
             "for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(), ros::names::resolve(scan_topic_).c_str());
  }
}

void AmclNode::requestMap()
{
    boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

    // get map via RPC
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the map...");
    while(!ros::service::call("static_map", req, resp))
    {
      ROS_WARN("Request for map failed; trying again...");
      ros::Duration d(0.5);
      d.sleep();
    }
    handleMapMessage( resp.map );
}

void AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if( first_map_only_ && first_map_received_ ) {
      return;
    }
    handleMapMessage( *msg );
    first_map_received_ = true;
}

void AmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
    ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
             msg.info.width, msg.info.height, msg.info.resolution);
    freeMapDependentMemory();
    // Clear queued laser objects because they hold pointers to the existing
    // map, #5202.
    lasers_.clear();
    lasers_update_.clear();
    frame_to_laser_.clear();

    map_ = convertMap(msg);

    scan_processor.GetMultiMap(map_);

#if NEW_UNIFORM_SAMPLING
    // Index of free space
    free_space_indices.resize(0);
    for(int i = 0; i < map_->size_x; i++)
        for(int j = 0; j < map_->size_y; j++)
          if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
            free_space_indices.push_back(std::make_pair(i,j));
#endif

    // Create the particle filter
    pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,(void *)map_);
    pf_->pop_err = pf_err_;
    pf_->pop_z = pf_z_;

    // Initialize the filter
    updatePoseFromServer();
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = init_pose_[0];
    pf_init_pose_mean.v[1] = init_pose_[1];
    pf_init_pose_mean.v[2] = init_pose_[2];
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = init_cov_[0];
    pf_init_pose_cov.m[1][1] = init_cov_[1];
    pf_init_pose_cov.m[2][2] = init_cov_[2];
    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
    pf_init_ = false;

    // Instantiate the sensor objects
    // Odometry
    delete odom_;
    odom_ = new AMCLOdom();
    ROS_ASSERT(odom_);
    odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
    // Laser
    delete laser_;
    laser_ = new AMCLLaser(max_beams_, map_);
    ROS_ASSERT(laser_);
    if(laser_model_type_ == LASER_MODEL_BEAM)
      laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,sigma_hit_, lambda_short_, 0.0);
    else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
      ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
      laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
                      laser_likelihood_max_dist_, do_beamskip_, beam_skip_distance_,
                      beam_skip_threshold_, beam_skip_error_threshold_);
      ROS_INFO("Done initializing likelihood field model.");
    }
    else
    {
      ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
      laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_);
      ROS_INFO("Done initializing likelihood field model.");
    }

    // In case the initial pose message arrived before the first map,
    // try to apply the initial pose now that the map has arrived.
    applyInitialPose();
}

void AmclNode::freeMapDependentMemory()
{
  if( map_ != NULL )
  {
    map_free( map_ );
    map_ = NULL;
  }
  if( pf_ != NULL )
  {
    pf_free( pf_ );
    pf_ = NULL;
  }
  delete odom_;
  odom_ = NULL;
  delete laser_;
  laser_ = NULL;
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates a map_t and returns it.
 */
map_t* AmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg)
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);

  unsigned long dat_size = map->size_x*map->size_y;


  for(int i=0; i<dat_size;i++)
  {
     map->cells[i].probability =((unsigned char)map_msg.data[i])/100.0;
  }

#if 1
  char *m=new char[dat_size];
  zw:: map_process(m,map_msg);

  for(int i=0;i< dat_size;i++)
  {
    if(m[i] == kFreeGrid){
      map->cells[i].occ_state = -1;
    }else if(m[i] == kOccGrid){
      map->cells[i].occ_state = +1;
    }else{
      map->cells[i].occ_state = 0;
    }
  }
  delete m;
#else
  for(int i=0; i<dat_size;i++)
  {
      if(map_msg.data[i]>=(char)(kOccProbaility*kOccGrid))
         map->cells[i].occ_state=1;
      else if(map_msg.data[i] <=(char)(kFreeprobaility*kOccGrid))
         map->cells[i].occ_state=-1;
      else
         map->cells[i].occ_state=0;
  }
#endif

  return map;
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void AmclNode::applyInitialPose()
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if( initial_pose_hyp_ != NULL && map_ != NULL ) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;

    delete initial_pose_hyp_;
    initial_pose_hyp_ = NULL;
  }
}

void AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  handleInitialPoseMessage(*msg);
}

void AmclNode::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if(msg.header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
    return;
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(tf_->resolve(msg.header.frame_id) != tf_->resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg.header.frame_id.c_str(), global_frame_id_.c_str());
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    ros::Time now = ros::Time::now();
    // wait a little for the latest tf to become available
    tf_->waitForTransform(base_frame_id_, msg.header.stamp,
                          base_frame_id_, now,
                          odom_frame_id_, ros::Duration(0.5));
    tf_->lookupTransform(base_frame_id_, msg.header.stamp,
                         base_frame_id_, now,
                         odom_frame_id_, tx_odom);
  }
  catch(tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(msg.pose.pose, pose_old);
  pose_new = pose_old * tx_odom;

  // Transform into the global frame
  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",ros::Time::now().toSec(),
           pose_new.getOrigin().x(),pose_new.getOrigin().y(),getYaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = getYaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  applyInitialPose();
}

double AmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void AmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  last_laser_received_ts_ = ros::Time::now();
  if( map_ == NULL )
    return;
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),
                                 ros::Time(), laser_scan->header.frame_id);
    tf::Stamped<tf::Pose> laser_pose;
    try{
      this->tf_->transformPose(base_frame_id_, ident, laser_pose);
    }catch(tf::TransformException& e){
      ROS_ERROR("Couldn't transform from %s to %s, ""even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(), base_frame_id_.c_str());
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0], laser_pose_v.v[1], laser_pose_v.v[2]);
    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  //  ROS_INFO("laser_index=%d",laser_index);

    //计算激光坐标系在全局坐标中的坐标
    scan_processor.SetLaserPose(laser_pose_v);
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_)){
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  if(pf_init_)
  {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > d_thresh_ || fabs(delta.v[1]) > d_thresh_ ||
                  fabs(delta.v[2]) > a_thresh_;
    update = update || m_force_update;
    m_force_update=false;
    // Set the laser update flags
    if(update)
      for(unsigned int i=0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;
    // Filter is now initialized
    pf_init_ = true;
    // Should update sensor data
    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;
    force_publication = true;
    resample_count_ = 0;
  }// If the robot has moved, update the filter
  else if(pf_init_ && lasers_update_[laser_index])
  {
    AMCLOdomData odata;
    odata.pose = pose;
    // HACK, Modify the delta in the action data so the filter gets updated correctly
    odata.delta = delta;
    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(lasers_update_[laser_index])
  {
    AMCLLaserData ldata;
    ldata.sensor = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();
    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    // Construct min and max angles of laser, in the base_link frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan->angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp,laser_scan->header.frame_id);
    q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
    tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp,laser_scan->header.frame_id);
    try{
      tf_->transformQuaternion(base_frame_id_, min_q, min_q);
      tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
    }catch(tf::TransformException& e){
      ROS_WARN("Unable to transform min/max laser angles into base frame: %s", e.what());
      return;
    }

    double angle_min = tf::getYaw(min_q);
    double angle_increment = tf::getYaw(inc_q) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;
    ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

    // Apply range min/max thresholds, if the user supplied them
    if(laser_max_range_ > 0.0)
      ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
    else
      ldata.range_max = laser_scan->range_max;
    double range_min;
    if(laser_min_range_ > 0.0)
      range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
    else
      range_min = laser_scan->range_min;
    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    ROS_ASSERT(ldata.ranges);
    for(int i=0;i<ldata.range_count;i++)
    {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if(laser_scan->ranges[i] <= range_min)
        ldata.ranges[i][0] = ldata.range_max;
      else
        ldata.ranges[i][0] = laser_scan->ranges[i];
      // Compute bearing
      ldata.ranges[i][1] = angle_min +(i * angle_increment);
    }
    // update weight, w_fast and w_slow
    lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
    lasers_update_[laser_index] = false;
    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }
    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud // TODO: set maximum rate for publishing
    if (!m_force_update) {
      geometry_msgs::PoseArray cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.poses.resize(set->sample_count);
      for(int i=0;i<set->sample_count;i++)
      {
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                        tf::Vector3(set->samples[i].pose.v[0],set->samples[i].pose.v[1], 0)),
                        cloud_msg.poses[i]);
      }
      particlecloud_pub_.publish(cloud_msg);
    }
  }


  if(resampled || force_publication)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0; hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      scan_match_pose_[0]=hyps[max_weight_hyp].pf_pose_mean.v[0];
      scan_match_pose_[1]=hyps[max_weight_hyp].pf_pose_mean.v[1];
      scan_match_pose_[2]=hyps[max_weight_hyp].pf_pose_mean.v[2];

      bool scan_flag =scan_processor.PoseUpdate(laser_scan, map_, scan_match_pose_);

      geometry_msgs::PoseStamped ptest;
      ptest.header.frame_id =global_frame_id_;
      ptest.header.stamp = laser_scan->header.stamp;
      ptest.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      ptest.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]), ptest.pose.orientation);
      pose_pub_amcl.publish(ptest);

      ptest.pose.position.x = scan_match_pose_[0];
      ptest.pose.position.y = scan_match_pose_[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(scan_match_pose_[2]), ptest.pose.orientation);
      pose_pub_scan.publish(ptest);

      if(scan_processor.publishScan)
         test.publish(scan_processor.ptcloud);

    if(add_close_loop && scan_flag)
    {
          pf_sample_set_t* mset = pf_->sets + pf_->current_set;
          for(int i=0 ; (i<50) && (mset->sample_count < pf_->max_samples) ; i++)
          {
              pf_sample_t *msample = mset->samples + mset->sample_count++;
              msample->pose ={scan_match_pose_[0],
                              scan_match_pose_[1],
                              scan_match_pose_[2]};

              for (int i = 0; i < mset->sample_count; i++)
              {
                msample = mset->samples + i;
                msample->weight = 1/mset->sample_count;
              }
          }
    //      ROS_INFO("sample count= %d",mset->sample_count);
    }
      //amcl or scan match ?
     pf_vector_t  finalPose ;
    if(use_amcl_pose)
    {
        finalPose = {hyps[max_weight_hyp].pf_pose_mean.v[0],
                     hyps[max_weight_hyp].pf_pose_mean.v[1],
                     hyps[max_weight_hyp].pf_pose_mean.v[2]};
    }else{
        finalPose = {scan_match_pose_[0],scan_match_pose_[1],scan_match_pose_[2]};
    }

  //  subMap_pub.publish(scan_processor.subMap);


      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = laser_scan->header.stamp;
      // Copy in the pose
   //   p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
   //   p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      p.pose.pose.position.x =finalPose.v[0];
      p.pose.pose.position.y =finalPose.v[1];

//      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
//                            p.pose.pose.orientation);
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(finalPose.v[2]),p.pose.pose.orientation);

      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      p.pose.covariance[6*5+5] = set->cov.m[2][2];
      pose_pub_.publish(p);
      last_published_pose = p;

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
//        tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
//                             tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
//                                         hyps[max_weight_hyp].pf_pose_mean.v[1],
//                                         0.0));
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(finalPose.v[2]),
                             tf::Vector3(finalPose.v[0], finalPose.v[1], 0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              laser_scan->header.stamp,
                                              base_frame_id_);
        this->tf_->transformPose(odom_frame_id_,tmp_tf_stamped,odom_to_map);
      }catch(tf::TransformException){
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
      }

      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;
      if(tf_broadcast_)
      {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        ros::Time transform_expiration = (laser_scan->header.stamp +transform_tolerance_);
        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration,
                                            global_frame_id_, odom_frame_id_);
        this->tfb_->sendTransform(tmp_tf_stamped);
        sent_first_transform_ = true;
      }
    }else{
      ROS_ERROR("No pose!");
    }
  }else if(latest_tf_valid_){
    if (tf_broadcast_)
    {

    // scan_match_pose_ = scan_processor.PoseUpdate(laser_scan, scan_match_pose_);

      // Nothing changed, so we'll just republish the last transform, to keep everybody happy.
      ros::Time transform_expiration = (laser_scan->header.stamp +transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration,
                                          global_frame_id_, odom_frame_id_);
      this->tfb_->sendTransform(tmp_tf_stamped);
    }
    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_period.toSec() > 0.0) && ((now - save_pose_last_time) >= save_pose_period))
    {
    //  this->savePoseToServer();
      save_pose_last_time = now;
    }
  }
}

bool AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,double& x, double& y, double& yaw,
                           const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)), t, f);
  try{
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }catch(tf::TransformException e){
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  return true;
}

bool AmclNode::setMapCallback(nav_msgs::SetMap::Request& req, nav_msgs::SetMap::Response& res)
{
  handleMapMessage(req.map);
  handleInitialPoseMessage(req.initial_pose);
  res.success = true;
  return true;
}

/*
 *  force nomotion updates (amcl updating without requiring motion)
*/
bool AmclNode::nomotionUpdateCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    m_force_update = true;
    //ROS_INFO("Requesting no-motion update");
    return true;
}

bool AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res)
{
  if( map_ == NULL ) {
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,(void *)map_);
  ROS_INFO("Global initialisation done!");
  pf_init_ = false;
  return true;
}

pf_vector_t AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  ROS_DEBUG("Generating new uniform sample");
  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}

void AmclNode::runFromBag(const std::string &in_bag_fn)
{
  rosbag::Bag bag;
  bag.open(in_bag_fn, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("tf"));
  std::string scan_topic_name = "base_scan"; // TODO determine what topic this actually is from ROS
  topics.push_back(scan_topic_name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Publisher laser_pub = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_name, 100);
  ros::Publisher tf_pub = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100);

  // Sleep for a second to let all subscribers connect
  ros::WallDuration(1.0).sleep();

  ros::WallTime start(ros::WallTime::now());

  // Wait for map
  while (ros::ok())
  {
    {
      boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
      if (map_)
      {
        ROS_INFO("Map is ready");
        break;
      }
    }
    ROS_INFO("Waiting for map...");
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0));
  }

  BOOST_FOREACH(rosbag::MessageInstance const msg, view)
  {
    if (!ros::ok())
    {
      break;
    }

    // Process any ros messages or callbacks at this point
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());

    tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg != NULL)
    {
      tf_pub.publish(msg);
      for (size_t ii=0; ii<tf_msg->transforms.size(); ++ii)
      {
        tf_->getBuffer().setTransform(tf_msg->transforms[ii], "rosbag_authority");
      }
      continue;
    }

    sensor_msgs::LaserScan::ConstPtr base_scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (base_scan != NULL)
    {
      laser_pub.publish(msg);
      laser_scan_filter_->add(base_scan);
      if (bag_scan_period_ > ros::WallDuration(0))
      {
        bag_scan_period_.sleep();
      }
      continue;
    }

    ROS_WARN_STREAM("Unsupported message type" << msg.getTopic());
  }

  bag.close();

  double runtime = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Bag complete, took %.1f seconds to process, shutting down", runtime);

  const geometry_msgs::Quaternion & q(last_published_pose.pose.pose.orientation);
  double yaw, pitch, roll;
  tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)).getEulerYPR(yaw,pitch,roll);
  ROS_INFO("Final location %.3f, %.3f, %.3f with stamp=%f",
            last_published_pose.pose.pose.position.x,
            last_published_pose.pose.pose.position.y,
            yaw, last_published_pose.header.stamp.toSec());

  ros::shutdown();
}

}
