#include "rotate_pid.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navit_auto_dock::plugins::RotatePidController,
                       navit_auto_dock::plugins::RotateController)
namespace navit_auto_dock {
namespace plugins {
void RotatePidController::initialize(std::string name,
                                     std::shared_ptr<tf2_ros::Buffer> &tf) {
  tf_buffer_ = tf;

  ros::NodeHandle pnh("~/" + name);

  std::string cmd_vel_topic_name = "cmd_vel";
  std::string odom_topic_name = "odom";

  pnh.param("cmd_vel_topic_name", cmd_vel_topic_name, cmd_vel_topic_name);
  pnh.param("odom_topic_name", odom_topic_name, odom_topic_name);
  pnh.param("rotate_timeout", config_.rotate_timeout, config_.rotate_timeout);
  pnh.param("Kp", config_.Kp, config_.Kp);
  pnh.param("max_vel", config_.max_vel, config_.max_vel);
  pnh.param("min_vel", config_.min_vel, config_.min_vel);
  pnh.param("control_frequency", config_.control_frequency, config_.control_frequency);
  pnh.param("yaw_tolerance", config_.yaw_tolerance, config_.yaw_tolerance);

  cmd_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 1);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
      odom_topic_name, 1, boost::bind(&RotatePidController::odomCallback, this, _1));

  ROS_INFO("Rotate pid controller is initialized");
}

bool RotatePidController::rotateToTarget(
    const geometry_msgs::PoseStamped &target) {
  target_ = target;

  ros::Rate r(config_.control_frequency);

  double yaw_error = 0.0;
  double yaw_vel = 0.0;
  ros::Time start = ros::Time::now();
  ros::Time current = ros::Time::now();

  while (ros::ok()) {
    current = ros::Time::now();
    yaw_error = getYawError();

    if (std::abs(yaw_error) < config_.yaw_tolerance) {
      cmd_vel_.angular.z = 0.0;
      cmd_pub_.publish(cmd_vel_);
      ROS_INFO("Rotate done");
      return true;
    }

    if ((current - start) > ros::Duration(config_.rotate_timeout)) {
      cmd_vel_.angular.z = 0.0;
      cmd_pub_.publish(cmd_vel_);
      ROS_WARN("Rotation timeout");
      return false;
    }
    ROS_DEBUG("yaw error is %f", yaw_error);
    yaw_vel = config_.Kp * yaw_error;

    if ( std::abs(yaw_vel) >= config_.max_vel ) yaw_vel = (yaw_vel > 0 ? 1 : -1) * config_.max_vel;
    if ( std::abs(yaw_vel) <= config_.min_vel ) yaw_vel = (yaw_vel > 0 ? 1 : -1) * config_.min_vel;

    cmd_vel_.angular.z = yaw_vel;
    cmd_pub_.publish(cmd_vel_);
    r.sleep();
  }

  cmd_vel_.angular.z = 0.0;
  cmd_pub_.publish(cmd_vel_);
  return false;
}

void RotatePidController::odomCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  current_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
}
}
}
