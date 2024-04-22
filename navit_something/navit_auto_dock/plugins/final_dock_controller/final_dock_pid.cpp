#include "final_dock_pid.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navit_auto_dock::plugins::PidController,
                       navit_auto_dock::plugins::FinalDockController)

namespace navit_auto_dock {
namespace plugins {
void PidController::initialize(std::string name,
                               std::shared_ptr<tf2_ros::Buffer> &tf) {
  tf_buffer_ = tf;

  ros::NodeHandle nh, pnh("~/" + name);

  // simulation works fine, might need to be tuned on actual robot
  pnh.param("Kp", config_.Kp, config_.Kp);
  pnh.param("tolerance", config_.tolerance, config_.tolerance);
  pnh.param("odom_topic_name", config_.odom_topic_name, config_.odom_topic_name);
  pnh.param("max_v", config_.max_v, config_.max_v);
  pnh.param("min_v", config_.min_v, config_.min_v);

  ROS_DEBUG("Kp is %f", config_.Kp);

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
      config_.odom_topic_name, 1, boost::bind(&PidController::odomCallback, this, _1));

  ROS_INFO("Final dock PID controller initailized!");
}

void PidController::setTargetPose(geometry_msgs::PoseStamped &target) {
  target_ = target;
  error_ = 0;
  odom_update_ = false;
}

bool PidController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    double cmd_v = config_.Kp * error_;
    clip(cmd_v);
    cmd_vel.linear.x = cmd_v;

  ROS_INFO("final dock error is %f , command is: %f", error_, cmd_vel.linear.x);
  return true;
}

bool PidController::isGoalReached() {
  if (std::abs(error_) < config_.tolerance && odom_update_)
    return true;
  return false;
}

void PidController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  current_.pose = msg->pose.pose;
  double theta = tf2::getYaw(msg->pose.pose.orientation);
  double diff_x = target_.pose.position.x - current_.pose.position.x;
  double diff_y = target_.pose.position.y - current_.pose.position.y;
  error_ = diff_x * std::cos(theta) + diff_y * std::sin(theta);
  odom_update_ = true;
}
}
}
