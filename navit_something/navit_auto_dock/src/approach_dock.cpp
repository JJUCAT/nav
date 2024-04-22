#include <navit_auto_dock/approach_dock.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace navit_auto_dock {

ApproachDock::ApproachDock(const std::string& name, 
                           std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                           std::shared_ptr<navit_collision_checker::CollisionChecker>& collision_checker,
                           ros::NodeHandle& nh): ApproachDock(name, tf_buffer, nh)
{
    ROS_WARN("ApproachDock with collision checker is deprecated!");
}

ApproachDock::ApproachDock(const std::string& name, 
                           std::shared_ptr<tf2_ros::Buffer> &tf_buffer, ros::NodeHandle& nh):
      nh_(nh),
      tf_buffer_(tf_buffer),
      approach_dock_as_(nh_, "/approach_dock",
                        boost::bind(&ApproachDock::executeCallback, this, _1),
                        false),
      approach_controller_pm_( name + "/controller",
                        boost::bind(&ApproachDock::loadControlPlugins, this, _1),
                        boost::bind(&ApproachDock::initControlPlugins, this, _1, _2), nh_),
      approach_perception_pm_( name + "/perception",
                        boost::bind(&ApproachDock::loadPerceptionPlugins, this, _1),
                        boost::bind(&ApproachDock::initPerceptionPlugins, this, _1, _2), nh_),
      approach_filter_pm_( name + "/filter",
                        boost::bind(&ApproachDock::loadFilterPlugins, this, _1),
                        boost::bind(&ApproachDock::initFilterPlugins, this, _1, _2), nh_),
      controller_loader_("navit_auto_dock",
                         "navit_auto_dock::plugins::ApproachDockController"),
      perception_loader_("navit_auto_dock", 
                         "navit_auto_dock::plugins::ApproachDockPerception"), 
      filter_loader_("navit_auto_dock",
                     "navit_auto_dock::plugins::ApproachDockFilter") 
{
  ros::NodeHandle pnh(name);


  pnh.param("controller_plugin",
            approach_dock_config.controller_name,
            approach_dock_config.controller_name);
  pnh.param("perception_plugin",
            approach_dock_config.perception_name,
            approach_dock_config.perception_name);
  pnh.param("filter_plugin",
            approach_dock_config.filter_name,
            approach_dock_config.filter_name);
  pnh.param("control_frequency",
            approach_dock_config.control_frequency,
            approach_dock_config.control_frequency);
  pnh.param("odom_topic_name",
            approach_dock_config.odom_topic_name,
            approach_dock_config.odom_topic_name);
  pnh.param("cmd_vel_name", approach_dock_config.cmd_vel_name,
            approach_dock_config.cmd_vel_name);

  pnh.param("obstacle_blocking_timeout", approach_dock_config.obstacle_blocking_timeout, approach_dock_config.obstacle_blocking_timeout);
  pnh.param("collision_check_forward_time", approach_dock_config.collision_check_forward_time, approach_dock_config.collision_check_forward_time);
  pnh.param("collision_check_forward_distance", approach_dock_config.collision_check_forward_distance, approach_dock_config.collision_check_forward_distance);
  pnh.param("lost_perception_timeout", approach_dock_config.lost_perception_timeout, approach_dock_config.lost_perception_timeout);
  pnh.param("global_frame", approach_dock_config.global_frame, approach_dock_config.global_frame);

  pnh.param("pre_in_percep_x", approach_dock_config.pre_in_percep_x_, approach_dock_config.pre_in_percep_x_);
  pnh.param("pre_in_percep_y", approach_dock_config.pre_in_percep_y_, approach_dock_config.pre_in_percep_y_);
  pnh.param("pre_in_percep_theta", approach_dock_config.pre_in_percep_theta_, approach_dock_config.pre_in_percep_theta_);

  // Collision checker with costmap
  costmap_ = std::make_shared<navit_costmap::Costmap2DROS>("auto_dock_costmap", *tf_buffer_, nh_);
  collision_checker_.reset(new navit_collision_checker::FootprintCollisionChecker< navit_costmap::Costmap2D*>(costmap_->getCostmap()));
  costmap_->pause();
  // Load plugins
  approach_controller_pm_.loadPlugins();
  approach_perception_pm_.loadPlugins();
  approach_filter_pm_.loadPlugins();

  // initialize default control, perception, filter plugins
  std::string default_controller_name = "dock_diff_graceful_controller/Controller";
  std::string default_perception_name = "dock_fake_perception/FakePerception";
  std::string default_filter_name = "PerceptionEKF";
  try
  {
      default_controller_ = controller_loader_.createInstance(default_controller_name);
      default_perception_ = perception_loader_.createInstance(default_perception_name);
      default_filter_ = filter_loader_.createInstance(default_filter_name);

      default_controller_->initialize(default_controller_name, tf_buffer_);
      default_perception_->initialize(default_perception_name, tf_buffer_);
      default_filter_->initialize(default_filter_name, tf_buffer_);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
     ROS_FATAL("Failed to create defualt plugins, Exception:%s", ex.what());
  }


  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>(approach_dock_config.cmd_vel_name, 1);
  odom_sub_ = global_nh.subscribe<nav_msgs::Odometry>( approach_dock_config.odom_topic_name, 1, boost::bind(&ApproachDock::odomCallback, this, _1));

  first_perception_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("first_percepted_dock_pose",1);
  dock_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("percepted_dock_pose",1);
  ref_dock_in_map_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ref_dock_in_map_pose",1);
  filtered_dock_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("filtered_dock_pose",1);
  precharging_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("precharing_pose",1);
  collision_check_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collision_check_poses",1);

  approach_dock_as_.start();
  ROS_INFO("Approach dock initialized!");
}

FilterPtr ApproachDock::loadFilterPlugins(const std::string& plugin)
{
    FilterPtr filter_ptr;
    try
    {
        filter_ptr = filter_loader_.createInstance(plugin);
        std::string filter_name = filter_loader_.getName(plugin);
        ROS_DEBUG_STREAM("Approach dock filter plugin " << plugin << " loaded");
    }
    catch (const pluginlib::PluginlibException& ex)
    {
        ROS_WARN_STREAM("Failed to load " << plugin << " as approach dock filter plugin"
                << ex.what());
    }

    return filter_ptr;
}

ControllerPtr ApproachDock::loadControlPlugins(const std::string& plugin)
{
    ControllerPtr controller_ptr;
    try
    {
        controller_ptr = controller_loader_.createInstance(plugin);
        std::string controller_name = controller_loader_.getName(plugin);
        ROS_DEBUG_STREAM("Approach dock controller plugin "<< plugin << " loaded");
    }
    catch(const pluginlib::PluginlibException& ex)
    {
        ROS_WARN_STREAM("Failed to load " << plugin << " as approach dock controller plugin"
                << ex.what());
    }

    return controller_ptr;
}

PerceptionPtr ApproachDock::loadPerceptionPlugins(const std::string& plugin)
{
    PerceptionPtr perception_ptr;
    try
    {
        //perception_ptr = boost::static_pointer_cast<plugins::ApproachDockPerception>(
        //        perception_loader_.createInstance(plugin));
        perception_ptr = perception_loader_.createInstance(plugin);
        std::string perception_name = perception_loader_.getName(plugin);
        ROS_DEBUG_STREAM("Approach dock perception plugin" << plugin << " loaded");
    }
    catch(const pluginlib::PluginlibException& ex)
    {
        ROS_WARN_STREAM("Failed to load " << plugin << " as approach dock controller plugin"
                << ex.what());
    }
    return perception_ptr;
}

bool ApproachDock::initFilterPlugins(const std::string& plugin,
                                     const FilterPtr& filter_ptr)
{
    //TODO: validate success?
    filter_ptr->initialize(plugin, tf_buffer_);
    return true;	
}

bool ApproachDock::initControlPlugins(const std::string& plugin,
                                      const ControllerPtr& controller_ptr)
{
    //TODO: validate if success
    controller_ptr->initialize(plugin, tf_buffer_);
    return true;
}

bool ApproachDock::initPerceptionPlugins(const std::string& plugin,
                                         const PerceptionPtr& perception_ptr)
{
    //TODO: validate if success
    perception_ptr->initialize(plugin, tf_buffer_);
    return true;
}

void ApproachDock::executeCallback( const navit_msgs::ApproachDockGoalConstPtr &goal) 
{
  ros::Rate r(approach_dock_config.control_frequency);
  //before start, we should check if the goal is valid
  double percepted_pose_offset = goal->percepted_pose_offset;

  if (percepted_pose_offset < 0) {
    ROS_ERROR("Approach dock pose should not be between robot pose and perception pose");
    result_.reached = false;
    approach_dock_as_.setAborted(result_, "Approach dock pose should not be between robot pose and perception pose");
  }

  offset_distance_ = -percepted_pose_offset;
  approach_dock_config.pre_in_percep_x_ = offset_distance_;

  // convert goal pose to global frame
  geometry_msgs::PoseStamped expected_dock_pose;
  geometry_msgs::PoseStamped final_expected_dock_pose;
  geometry_msgs::PoseStamped pre_charging_pose;
  tf2::Transform dock_to_target_tf;

  // TODO(czk) :(poseStamped -> pose) We should not use the timestamp of the goal pose, because it may be too old
  geometry_msgs::PoseStamped current_pose_on_map;
  current_pose_on_map = goal->expected_dock_pose; // 这里的充电桩位姿是软件保存的 map 下位姿
  ref_dock_in_map_pub_.publish(current_pose_on_map);
  current_pose_on_map.header.stamp = ros::Time::now();

  toGlobalFrame(current_pose_on_map, expected_dock_pose); // 把充电桩位姿转到 odom 坐标下
  ROS_INFO("expected pose x %f y %f yaw %f", expected_dock_pose.pose.position.x,
    expected_dock_pose.pose.position.y, tf2::getYaw(expected_dock_pose.pose.orientation));
  // toGlobalFrame(goal->pre_charging_pose, pre_charging_pose);
  calcPreposeFromPercpose(expected_dock_pose, pre_charging_pose); // 获取 odom 下的回充前置点
  ROS_INFO("ApproachDock: Goal received");

  // collision check
  if (collision_checker_ == nullptr)
  {
    ROS_WARN("collision checker is not initilized");
    result_.reached = false;
    approach_dock_as_.setAborted(result_, "collision checker not initilized");
    return;
  }

  // get controller
  // TODO: validate plugin name
  if (goal->controller_plugin_name == "")
  {
     ROS_WARN("Requested controller is empty! use default controller");
     controller_ = default_controller_;
  }
  else
  {
     if (approach_controller_pm_.hasPlugin(goal->controller_plugin_name))
     {
        controller_ = approach_controller_pm_.getPlugin(goal->controller_plugin_name);
     }
     else
     {
        ROS_FATAL("Requested controller %s doesn't exist!", (goal->controller_plugin_name).c_str() );
        publishZeroCmdVel();
        result_.reached = false;
        approach_dock_as_.setAborted(result_, "controller plugin dose not exist");
        return;
     }
  }

  if (goal->perception_plugin_name == "")
  {
     ROS_WARN("Requested perception plugin is empty! use default plugin");
     perception_ = default_perception_;
  }
  else
  {
     if (approach_perception_pm_.hasPlugin(goal->perception_plugin_name))
     {
        perception_ = approach_perception_pm_.getPlugin(goal->perception_plugin_name);
     }
     else
     {
        ROS_FATAL("Requested perception %s doesn't exist!", (goal->perception_plugin_name).c_str() );
        publishZeroCmdVel();
        result_.reached = false;
        approach_dock_as_.setAborted(result_, "perception plugin dose not exist");
        return;
     }
  }

  if (goal->filter_plugin_name == "")
  {
     ROS_WARN("Requested filter plugin is empty! use default plugin");
     filter_ = default_filter_;
  }
  else
  {
      if (approach_filter_pm_.hasPlugin(goal->filter_plugin_name))
      {
        filter_ = approach_filter_pm_.getPlugin(goal->filter_plugin_name);
      }
      else
      {
        ROS_FATAL("Requested filter %s doesn't exist!", (goal->filter_plugin_name).c_str() );
        publishZeroCmdVel();
        result_.reached = false;
        approach_dock_as_.setAborted(result_, "filter plugin dose not exist");
        return;
      }
  }

  ROS_DEBUG_STREAM("controller plugin "<<goal->controller_plugin_name);
  ROS_DEBUG_STREAM("perception plugin "<<goal->perception_plugin_name);
  ROS_DEBUG_STREAM("filter plugin "<<goal->filter_plugin_name);
  result_.reached = false;

  // start perception
  perception_->start(expected_dock_pose);
  sleep(1);

  ros::Time last_nonblock_time = ros::Time::now();
  ros::Duration block_time;
  ros::Time last_percepted_time = ros::Time::now();
  ros::Duration lost_perception_time;

  // initialize filter
  filter_->reset();

  sleep(2); // 点云累积

  // wait for costmap back online
  ROS_WARN("[AD] wait for costmap resume ...");
  costmap_->resume();
  while (!costmap_->isCurrent()) r.sleep();

  // main loop
  bool find_pre_charing = false, first_perception = true;
  double control_time = 0.0, control_time_max = -1;
  int control_count = 0, control_max_count = 0;
  double perception_time = 0.0, perception_time_max = -1;
  int perception_count = 0, perception_max_count = 0, perception_succeed_count = 0;
  while (ros::ok()) {
    if (approach_dock_as_.isPreemptRequested()) {
      publishZeroCmdVel();
      ROS_WARN("[AD] apparoch_dock is preempted!");
      approach_dock_as_.setPreempted(result_);
      break;
    }

    if (controller_->isGoalReached()) {
      publishZeroCmdVel();
      expected_dock_pose = WaitAndUpdateDock(expected_dock_pose, 2.0f);

      result_.reached = true;
      result_.dock_pose = expected_dock_pose;
      approach_dock_as_.setSucceeded(result_, "Pose reached!");
      ROS_INFO("[AD] ApproachDock: Goal Reached!");

      ROS_WARN("[AD] final expected dock pose [%s][%f,%f,%f]",
        final_expected_dock_pose.header.frame_id.c_str(),
        final_expected_dock_pose.pose.position.x, final_expected_dock_pose.pose.position.y,
        tf2::getYaw(final_expected_dock_pose.pose.orientation));

      ROS_INFO("[AD] perception speed %f s, max time %f s, count %d, succeed count %d (%.3f)",
        perception_time / perception_count, perception_time_max, perception_count, perception_succeed_count, static_cast<float>(perception_succeed_count)/perception_count);
      ROS_INFO("[AD] control calculate speed %f s, max time %f s, count %d, more than 0.1s count %d (%.3f)",
        control_time / control_count, control_time_max, control_count, control_max_count, static_cast<float>(control_max_count)/control_count);

      break;
    }

    perception_count++;
    auto stp = ros::Time::now().toSec();
    if (perception_->getPose(expected_dock_pose)) {
      perception_succeed_count++;
      final_expected_dock_pose = expected_dock_pose;
      ROS_INFO("[AD] expected dock pose [%s][%f,%f,%f]",
        expected_dock_pose.header.frame_id.c_str(),
        expected_dock_pose.pose.position.x, expected_dock_pose.pose.position.y,
        tf2::getYaw(expected_dock_pose.pose.orientation));
      if (first_perception) {
        first_perception = false;
        first_perception_pub_.publish(expected_dock_pose);
      }
      last_percepted_time = ros::Time::now();

      filter_->update(expected_dock_pose, current_vel_);
      filtered_dock_pose_pub_.publish(expected_dock_pose);
      ROS_INFO("[AD] after filter dock [%s][%f,%f,%f]",
        expected_dock_pose.header.frame_id.c_str(),
        expected_dock_pose.pose.position.x, expected_dock_pose.pose.position.y,
        tf2::getYaw(expected_dock_pose.pose.orientation));

      calcPreposeFromPercpose(expected_dock_pose,pre_charging_pose);
      ROS_INFO("[AD] get precharging pose [%s][%f,%f,%f]",
        expected_dock_pose.header.frame_id.c_str(),
        pre_charging_pose.pose.position.x, pre_charging_pose.pose.position.y,
        tf2::getYaw(pre_charging_pose.pose.orientation));

      if (perception_succeed_count > 1) {
        find_pre_charing = true;
      }
      else continue;
    } else {
      lost_perception_time = ros::Time::now() - last_percepted_time;
      if (lost_perception_time.toSec() > approach_dock_config.lost_perception_timeout)
      {
          publishZeroCmdVel();
          ROS_ERROR("[AD] Perception plugin could not get Pose to long (for %f seconds)",approach_dock_config.lost_perception_timeout);
          result_.reached = false;
          approach_dock_as_.setAborted(result_, "perception lost timeout, failed to find the dock");
          break;
      }
      ROS_INFO_STREAM_THROTTLE(1.0,"[AD] Perception plugin could not get Pose, lost time is " << lost_perception_time.toSec() << " seconds");
      if (!find_pre_charing) continue; // 一直获取不到回充前置点，就等着
      // filter_->update(expected_dock_pose, current_vel_);
      // calcPreposeFromPercpose(expected_dock_pose, pre_charging_pose);
    }
    auto etp = ros::Time::now().toSec();
    perception_time += etp-stp;
    if (etp-stp > 0.1) perception_max_count ++;
    if (etp-stp> perception_time_max) perception_time_max = etp-stp;

    dock_pose_pub_.publish(expected_dock_pose);
    precharging_pose_pub_.publish(pre_charging_pose);
    controller_->setTarget(pre_charging_pose);

    // compute control commands
    auto stc = ros::Time::now().toSec();
    control_count++;
    if (controller_->computeVelocityCommands(current_pose_, current_vel_, cmd_vel_)) {
        if ( collisionCheckForwardPath(approach_dock_config.collision_check_forward_distance,
                            approach_dock_config.collision_check_forward_time) )
        {
            ROS_WARN("[AD] check collision forward path");
            block_time = ros::Time::now() - last_nonblock_time;

            if (block_time.toSec() < approach_dock_config.obstacle_blocking_timeout )
            {
                ROS_WARN("Possible collision detected!Stop!");
                publishZeroCmdVel();
            }
            else
            {
                publishZeroCmdVel();
                result_.reached = false;
                approach_dock_as_.setAborted(result_, "target is blocked, abort the mission");
                break;
            }
        }
        else
        {
            auto cmd = cmd_vel_;
            cmd = SlowDown(pre_charging_pose, cmd_vel_, 8, 0.08, 0.06);
            cmd_vel_pub_.publish(cmd_vel_);
            last_nonblock_time = ros::Time::now();
        }
    } else {
      ROS_WARN("failed to compute command vels");
      publishZeroCmdVel();
    }
    auto etc = ros::Time::now().toSec();
    control_time += etc-stc;
    if (etc-stc > 0.1) control_max_count ++;
    if (etc-stc > control_time_max) control_time_max = etc-stc;

    feedback_.dock_pose = expected_dock_pose;
    feedback_.target_pose = pre_charging_pose;
    approach_dock_as_.publishFeedback(feedback_);

    r.sleep();
  }

  // save some cpu
  costmap_->pause();
  perception_->stop();
}

bool ApproachDock::toChargePile(const geometry_msgs::PoseStamped& dock_pose,
                                      geometry_msgs::PoseStamped& charge_port_pose) { 
  std::string charge_pile_frame = "base_charge_port";
  try {
    geometry_msgs::TransformStamped tf_pose_to_odom =
        tf_buffer_->lookupTransform(
            charge_pile_frame, "base_link", ros::Time(0));
    tf2::doTransform(dock_pose, charge_port_pose, tf_pose_to_odom);
    charge_port_pose.header.frame_id = "odom";
  } catch (const tf2::TransformException &ex) {
    ROS_FATAL("Failed to transform to global frame %s", ex.what());
    return false;
  }
  return true;
}
void ApproachDock::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
  current_pose_.header.frame_id = odom_msg->header.frame_id;
  current_pose_.pose = odom_msg->pose.pose;
  current_vel_ = odom_msg->twist.twist;
}

inline void ApproachDock::publishZeroCmdVel() {
  cmd_vel_ = geometry_msgs::Twist();
  cmd_vel_.linear.x=0.0;
  cmd_vel_.linear.y=0.0;
  cmd_vel_.linear.z=0.0;
  cmd_vel_.angular.z=0.0;
  // ROS_INFO("-----publishZeroCmdVel: cmd_vel_.linear.x=%lf cmd_vel_.angular=%lf-----",cmd_vel_.linear.x,cmd_vel_.angular.z);
  cmd_vel_pub_.publish(cmd_vel_);
}

void ApproachDock::toGlobalFrame(const geometry_msgs::PoseStamped &local_pose,
                                 geometry_msgs::PoseStamped &global_pose) {
  global_pose.header.frame_id = approach_dock_config.global_frame;
  if (local_pose.header.frame_id != approach_dock_config.global_frame) {
    try {
      tf_buffer_->transform(local_pose, global_pose,
                            approach_dock_config.global_frame, ros::Duration(0.5));
      ROS_DEBUG("to global frame");
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Failed to transform from %s to %s", local_pose.header.frame_id.c_str(),
                approach_dock_config.global_frame.c_str());
      ROS_WARN_STREAM("Failed to transform from "
                  << local_pose.header.frame_id << " to "
                  << approach_dock_config.global_frame << ": " << ex.what());
    }
  } else {
    global_pose = local_pose;
  }
}

void ApproachDock::getDockToTargetTF(const geometry_msgs::PoseStamped &dock, 
                                     const geometry_msgs::PoseStamped &target,
                                     tf2::Transform &dock_to_target_tf){
    tf2::Transform odom_to_dock_tf, odom_to_target_tf;
    tf2::fromMsg(dock.pose, odom_to_dock_tf);
    tf2::fromMsg(target.pose, odom_to_target_tf);
    

    geometry_msgs::PoseStamped dock_test, target_test; 
    tf2::toMsg(odom_to_dock_tf, dock_test.pose);

    dock_to_target_tf = odom_to_dock_tf.inverse() * odom_to_target_tf; 
    tf2::toMsg(odom_to_dock_tf * dock_to_target_tf, target_test.pose);
    ROS_WARN("dock pose is {%f,%f,%f}, target pose is {%f,%f,%f}", dock_test.pose.position.x,
                                                                   dock_test.pose.position.y,
                                                                   tf2::getYaw(dock_test.pose.orientation),
                                                                   target_test.pose.position.x,
                                                                   target_test.pose.position.y,
                                                                   tf2::getYaw(target_test.pose.orientation));

}
// TODO: to be verified
// calculate pre_charging_pose based on new detected pose and fixed tf
void ApproachDock::fixTargetPose(const geometry_msgs::PoseStamped &dock,
                                 const tf2::Transform &dock_to_target_tf,
                                 geometry_msgs::PoseStamped &target){
    tf2::Transform odom_to_dock_tf, odom_to_target_tf;
    tf2::fromMsg(dock.pose, odom_to_dock_tf);

    odom_to_target_tf = odom_to_dock_tf * dock_to_target_tf;
    ROS_DEBUG("deteceted dock pose is {%f,%f,%f}", dock.pose.position.x,
                                                  dock.pose.position.y,
                                                  tf2::getYaw(dock.pose.orientation));
    ROS_DEBUG("before detection, target pose is {%f,%f,%f}", target.pose.position.x,
                                                            target.pose.position.y,
                                                            tf2::getYaw(target.pose.orientation));
    geometry_msgs::PoseStamped target_fix;
    tf2::toMsg(odom_to_target_tf, target.pose);

    ROS_DEBUG("after detection, target pose is {%f,%f,%f}", target.pose.position.x,
                                                           target.pose.position.y,
                                                           tf2::getYaw(target.pose.orientation));
}

void ApproachDock::calcPreposeFromPercpose(
  const geometry_msgs::PoseStamped &perception_dock,
  geometry_msgs::PoseStamped &pre_charging_pose) {
  tf2::Transform odom_to_preception_tf, odom_to_precharging_tf;
  tf2::fromMsg(perception_dock.pose,odom_to_preception_tf);

  tf2::Transform perceptin_to_precharging_tf;
  perceptin_to_precharging_tf.setOrigin(tf2::Vector3(
    approach_dock_config.pre_in_percep_x_, approach_dock_config.pre_in_percep_y_, 0.0));
  tf2::Quaternion q;  
  q.setRPY(0.0, 0.0, approach_dock_config.pre_in_percep_theta_); 
  perceptin_to_precharging_tf.setRotation(q);

  odom_to_precharging_tf=odom_to_preception_tf*perceptin_to_precharging_tf;
  tf2::toMsg(odom_to_precharging_tf, pre_charging_pose.pose);
  pre_charging_pose.header=perception_dock.header;
  ROS_DEBUG_NAMED("PRECHARGING_POSE","[X:%f,Y:%f,Z:%f,QUATERNION:%f,%f,%f,%f]",
    pre_charging_pose.pose.position.x, pre_charging_pose.pose.position.y,
    pre_charging_pose.pose.position.z, pre_charging_pose.pose.orientation.x,
    pre_charging_pose.pose.orientation.y, pre_charging_pose.pose.orientation.z,
    pre_charging_pose.pose.orientation.w);
}

bool ApproachDock::collisionCheckForwardPath(double forward_distance, double forward_time)
{
  geometry_msgs::PoseStamped check_pose;
  geometry_msgs::PoseStamped current_pose_on_map;
  if(current_pose_.header.frame_id==" "){
    current_pose_.header.frame_id=="odom";
  }
  try {
    tf_buffer_->transform(current_pose_, current_pose_on_map, costmap_->getGlobalFrameID());
    ROS_INFO_STREAM_ONCE("success to transform from "
      << current_pose_.header.frame_id << " to "
      << costmap_->getGlobalFrameID()<<" " << "frame");
  } catch (tf2::TransformException &ex) {
    ROS_WARN_STREAM("Failed to transform from "
      << current_pose_.header.frame_id << " to "
      << costmap_->getGlobalFrameID());
  }
  ROS_DEBUG("current_pose_on_map:( %lf,%lf) \n",current_pose_on_map.pose.position.x,current_pose_on_map.pose.position.y);
  check_pose = current_pose_on_map;

  double dt = 0.3, d = 0;
  int N = std::max(static_cast<int>(std::ceil(forward_time/dt)),2);
  double diff_x = cmd_vel_.linear.x * dt;
  double diff_y = cmd_vel_.linear.y * dt;
  double diff_yaw = cmd_vel_.angular.z * dt;
  double heading = tf2::getYaw(current_pose_.pose.orientation);

  geometry_msgs::PoseArray collision_check_poses;
  collision_check_poses.header = check_pose.header;
  for (int n = 0; n < N; n++) {
    geometry_msgs::PoseStamped next_pose = check_pose;
    double yaw = fabs(diff_yaw) * (cmd_vel_.linear.x * cmd_vel_.angular.z > 0 ? 1 : -1);
    heading += yaw;
    next_pose.pose.position.x += diff_x * std::cos(heading) - diff_y * std::sin(heading);
    next_pose.pose.position.y += diff_x * std::sin(heading) + diff_y * std::cos(heading);
    tf2::Quaternion q; q.setRPY(0, 0, heading);
    next_pose.pose.orientation = tf2::toMsg(q);

    if (collisionCheckPose(next_pose)) {
      ROS_WARN("ApproachDock detects collision at Pose (%.2f, %.2f)!", 
        check_pose.pose.position.x, check_pose.pose.position.y);
      return true;
    }

    d += std::hypot(diff_x, diff_y);
    if (d > forward_distance) {
      ROS_DEBUG("d %lf ,forward_distance is  %f", d, forward_distance);
      break;
    }

    check_pose = next_pose;
    collision_check_poses.poses.push_back(check_pose.pose);    
  }
  collision_check_poses_pub_.publish(collision_check_poses);
  return false; 
}

bool ApproachDock::LogOdomFrameInMapFrame(geometry_msgs::PoseStamped& odom_in_map) {
  std::string local_frame = "odom";
  geometry_msgs::PoseStamped odom;
  odom.header.frame_id = local_frame;
  odom.header.stamp = ros::Time::now();
  tf2::Quaternion q; q.setRPY(0.0, 0.0, 0.0);
  odom.pose.orientation = tf2::toMsg(q);

  try {
    tf_buffer_->transform(odom, odom_in_map, "map");
    ROS_INFO("[AD] odom in map [%f, %f, %f]",
      odom_in_map.pose.position.x, odom_in_map.pose.position.y,
      tf2::getYaw(odom_in_map.pose.orientation));
  } catch (const tf2::TransformException& ex) {
    ROS_WARN("[AD] get odom in map failed !");
    return false;
  }
  return true;
}

geometry_msgs::Twist ApproachDock::SlowDown(const geometry_msgs::PoseStamped& precharging_pose,
  const geometry_msgs::Twist& cmd_vel, const double slow_down_rang, const double v_max, const double v_min)
{
  geometry_msgs::PoseStamped precharging_map;
  toGlobalFrame(precharging_pose, precharging_map);
  double d = std::hypot(precharging_map.pose.position.x - current_pose_.pose.position.x,
                        precharging_map.pose.position.y - current_pose_.pose.position.y);
  geometry_msgs::Twist slow_cmd = cmd_vel;
  if (d < slow_down_rang) slow_cmd.linear.x = v_max * d / slow_down_rang;
  // slow_cmd.linear.x = std::max(std::min(slow_cmd.linear.x, fabs(cmd_vel.linear.x)), v_min);
  slow_cmd.linear.x = v_min;
  slow_cmd.linear.x *= cmd_vel.linear.x < 0 ? -1 : 1;
  return slow_cmd;
}

geometry_msgs::PoseStamped ApproachDock::WaitAndUpdateDock(
  const geometry_msgs::PoseStamped dock, const double wait_time)
{
  ROS_INFO("[AD] wait and update dock.");
  ros::Time st = ros::Time::now();
  geometry_msgs::PoseStamped expected_dock_pose = dock;
  ros::Rate r(approach_dock_config.control_frequency);
  while(ros::ok()) {
    if ((ros::Time::now()-st).toSec() > wait_time) {
      break;
    }

    if (perception_->getPose(expected_dock_pose)) {
      ROS_INFO("[AD] wait&update expected dock pose [%s][%f,%f,%f]",
        expected_dock_pose.header.frame_id.c_str(),
        expected_dock_pose.pose.position.x, expected_dock_pose.pose.position.y,
        tf2::getYaw(expected_dock_pose.pose.orientation));

      filter_->update(expected_dock_pose, current_vel_);
      filtered_dock_pose_pub_.publish(expected_dock_pose);
      ROS_INFO("[AD] wait&update after filter dock [%s][%f,%f,%f]",
        expected_dock_pose.header.frame_id.c_str(),
        expected_dock_pose.pose.position.x, expected_dock_pose.pose.position.y,
        tf2::getYaw(expected_dock_pose.pose.orientation));
    }
    r.sleep();
  }
  ROS_INFO("[AD] ajust dock distance %f",
    std::hypot(expected_dock_pose.pose.position.x-dock.pose.position.x,
               expected_dock_pose.pose.position.y-dock.pose.position.y));
  return expected_dock_pose;
}

}
