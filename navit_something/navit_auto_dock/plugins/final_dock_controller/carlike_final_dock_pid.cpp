#include "carlike_final_dock_pid.h"

#include <pluginlib/class_list_macros.h>
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"

PLUGINLIB_EXPORT_CLASS(navit_auto_dock::plugins::CarlikePidController,
                       navit_auto_dock::plugins::FinalDockController)

namespace navit_auto_dock {
namespace plugins {
void CarlikePidController::initialize(std::string name,
                               std::shared_ptr<tf2_ros::Buffer> &tf) {
  tf_buffer_ = tf;

  ros::NodeHandle nh, pnh("~/" + name);

  pnh.param("odom_topic_name", config_.odom_topic_name, config_.odom_topic_name);
  pnh.param("steering_max", config_.steering_max, config_.steering_max);
  pnh.param("kp_y_err", config_.kp_y_err, config_.kp_y_err);
  pnh.param("kp_theta_err", config_.kp_theta_err, config_.kp_theta_err);
  pnh.param("tolerance", config_.tolerance, config_.tolerance);
  pnh.param("max_v", config_.max_v, config_.max_v);
  pnh.param("min_v", config_.min_v, config_.min_v);
  pnh.param("target_x_offset_to_dock", config_.target_x_offset_to_dock, config_.target_x_offset_to_dock);
  pnh.param("hook", config_.hook, config_.hook);

  pnh.param("lidar/topic_name", lidar_config_.topic_name, lidar_config_.topic_name);
  pnh.param("lidar/valid_x", lidar_config_.valid_x, lidar_config_.valid_x);
  pnh.param("lidar/x_min", lidar_config_.x_min, lidar_config_.x_min);
  pnh.param("lidar/x_max", lidar_config_.x_max, lidar_config_.x_max);
  pnh.param("lidar/y_min", lidar_config_.y_min, lidar_config_.y_min);
  pnh.param("lidar/y_max", lidar_config_.y_max, lidar_config_.y_max);
  pnh.param("lidar/z_min", lidar_config_.z_min, lidar_config_.z_min);
  pnh.param("lidar/z_max", lidar_config_.z_max, lidar_config_.z_max);
  pnh.param("lidar/i_min", lidar_config_.i_min, lidar_config_.i_min);
  pnh.param("lidar/i_max", lidar_config_.i_max, lidar_config_.i_max);

  ROS_INFO("Final dock PID controller initailized!");
}

void CarlikePidController::setTargetPose(geometry_msgs::PoseStamped &target) {
  target_ = target;

  odom_update_ = false;
  x_in_lidar_ = std::numeric_limits<double>::lowest();
  journey_ = 0;
  journey_history_ = 0;
  target_distance_ = std::numeric_limits<double>::lowest();

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      config_.odom_topic_name, 1, boost::bind(&CarlikePidController::odomCallback, this, _1));
  if (config_.target_x_offset_to_dock > 0) {
    lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        lidar_config_.topic_name, 1, boost::bind(&CarlikePidController::lidarCallback, this, _1));  
    sleep(2);
    updateTargetDistance();          
  }
}

bool CarlikePidController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!odom_update_) return false;
  if (!updatePoseInTarget(target_)) return false;

  updateTargetDistance();
  updateJourney();
  double d = target_distance_-journey_;
  ROS_DEBUG("[CarFD] remain distance %f", d);

  // steering(t) = kp_theta * theta_e(t) + kp_y * arctan(k * e(t))
  double theta_err = getThetaToDockRefLine();
  double y_err = getDistanceToDockRefLine();
  // double limit = 6;
  // double v = fabs(d) / limit * (config_.max_v - config_.min_v) + config_.min_v;
  double v = config_.min_v * getDirectionToDockRefLine();
  int sign = y_err < 0 ? 1 : -1;
  double steering;
  if (getDirectionToDockRefLine() > 0) {
    // steering = config_.kp_theta_err * theta_err * -1 + config_.kp_y_err * fabs(y_err) * sign;
    steering = config_.kp_theta_err * theta_err * -1 + config_.kp_y_err * fabs(atan(y_err)) * sign;
  } else {
    steering = config_.kp_theta_err * theta_err + config_.kp_y_err * fabs(atan(y_err)) * sign;
  }

  steering = std::max(std::min(steering, config_.steering_max), -config_.steering_max);

  cmd_vel.linear.x = v;
  cmd_vel.angular.z = steering;

  return true;
}

bool CarlikePidController::isGoalReached() {
  if (fabs(target_distance_-journey_) < config_.tolerance) {
    odom_sub_.shutdown();
    if (config_.target_x_offset_to_dock > 0) lidar_sub_.shutdown();
    ROS_INFO("[CarFD] final y err %f, theta err %f", getDistanceToDockRefLine(), getThetaToDockRefLine());
    return true;
  }
  return false;
}

void CarlikePidController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  current_.pose = msg->pose.pose;
  odom_ = *msg;
  if (!odom_update_) last_odom_ = odom_;
  odom_update_ = true;
}

void CarlikePidController::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  if (!odom_update_) return;

  sensor_msgs::PointCloud2 pc=*msg;
  try {
    geometry_msgs::TransformStamped tf;
    tf = tf_buffer_->lookupTransform(
        "base_link", msg->header.frame_id, ros::Time(0));
    tf2::doTransform(pc, pc, tf);
    pc.header.frame_id="base_link";
  } catch (tf2::TransformException const &ex) {
    ROS_WARN_STREAM_THROTTLE(1.0, "Couldn't transform origin scan to baselink frame");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI> pcl_pc;
  pcl::fromROSMsg(pc, pcl_pc);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pcl::PointCloud<pcl::PointXYZI> pc_filtered;
  pass.setInputCloud (pcl_pc.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (lidar_config_.z_min,lidar_config_.z_max);
  pass.filter (pc_filtered);

  pass.setInputCloud (pc_filtered.makeShared());
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (lidar_config_.x_min, lidar_config_.x_max);
  pass.filter (pc_filtered);

  pass.setInputCloud (pc_filtered.makeShared());
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (lidar_config_.y_min, lidar_config_.y_max);
  pass.filter (pc_filtered);

  pass.setInputCloud (pc_filtered.makeShared());
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (lidar_config_.i_min, lidar_config_.i_max);
  pass.filter (pc_filtered);

  if (pc_filtered.points.size() < 10) {
    // ROS_WARN("[CarFD] lose dock");
    return;
  }

  pcl::PointXYZI min, max;
  pcl::getMinMax3D(pc_filtered,min,max);
  ROS_INFO("[CarFD] dock lidar x_min:%f, x_max:%f", min.x, max.x);

  ros::Time now = ros::Time::now();
  double v = odom_.twist.twist.linear.x;
  v = v < 0 ?  0 : v;
  ros::Duration lidar_delay = now - msg->header.stamp;
  ROS_WARN("[CarFD] lidar delay %f", lidar_delay.toSec());
  x_min_buffer_.push_back(min.x - lidar_delay.toSec() * v);
  if (!x_min_buffer_.empty()) {
    double predicted_x = x_in_lidar_ - (now - lidar_callback_time_).toSec() * v;
    x_min_buffer_.push_back(predicted_x);
  }
  std::sort(x_min_buffer_.begin(), x_min_buffer_.end());
  x_min_buffer_.resize(lidar_config_.buffer_size);
  lidar_callback_time_ = now;

  std::lock_guard<std::mutex> lock(lidar_mutex_);
  x_in_lidar_ = x_min_buffer_.front();
}

bool CarlikePidController::updatePoseInTarget(const geometry_msgs::PoseStamped& target) {
  if (!odom_update_) return false;
  tf2::Transform odom_in_target, target_in_odom, robot_in_odom, robot_in_target;
  tf2::fromMsg(target.pose, target_in_odom);
  odom_in_target = target_in_odom.inverse();

  tf2::fromMsg(odom_.pose.pose, robot_in_odom);
  
  robot_in_target = odom_in_target*robot_in_odom;
  geometry_msgs::PoseStamped robot;
  tf2::toMsg(robot_in_target, pose_in_target_.pose);

  return true;
}

double CarlikePidController::getDistanceToDockRefLine() {
  double y_err = pose_in_target_.pose.position.y;
  y_err = y_err + config_.hook * sin(getThetaToDockRefLine());
  return y_err;
}

double CarlikePidController::getThetaToDockRefLine() {
  return tf2::getYaw(pose_in_target_.pose.orientation);
}

int CarlikePidController::getDirectionToDockRefLine() {
  return pose_in_target_.pose.position.x < 0 ? 1 : -1;
}

void CarlikePidController::updateTargetDistance() {
  if (target_distance_ < 0) {
    target_distance_ = std::hypot(target_.pose.position.x - odom_.pose.pose.position.x,
                                  target_.pose.position.y - odom_.pose.pose.position.y);
    journey_ = 0;
    // last_odom_ = odom_;
    ROS_INFO("[CarFD] reset target dist %f", target_distance_);
  }
  
  if ((config_.target_x_offset_to_dock > 0) && (getDirectionToDockRefLine() > 0)) {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    if (x_in_lidar_ > lidar_config_.valid_x + config_.target_x_offset_to_dock) {
      target_distance_ = x_in_lidar_ - config_.target_x_offset_to_dock;
      journey_ = 0;
      // last_odom_ = odom_;
      ROS_INFO("[CarFD] reset target dist %f by lidar, valid x %f, target offset %f",
        target_distance_, lidar_config_.valid_x, config_.target_x_offset_to_dock);
    }
  }
}

void CarlikePidController::updateJourney() {
  double d = std::hypot(last_odom_.pose.pose.position.x - odom_.pose.pose.position.x,
                        last_odom_.pose.pose.position.y - odom_.pose.pose.position.y);
  journey_ += d;
  journey_history_ += d;
  last_odom_ = odom_;
  ROS_DEBUG("[CarFD] journey history %f, journey %f", journey_history_, journey_);
}

}
}
