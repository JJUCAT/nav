#include <dock_diff_graceful_controller/controller.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dock_diff_graceful_controller::Controller,
                       navit_auto_dock::plugins::ApproachDockController)

namespace dock_diff_graceful_controller {

void Controller::initialize(const std::string name,
                            const std::shared_ptr<tf2_ros::Buffer> &tf) {
  tf_buffer_ = tf;
  ros::NodeHandle pnh("~/" + name);

  // TODO(enhancement): add dynamic reconfig server for these
  k1_ = 1;
  k2_ = 10;
  min_velocity_ = 0.15;
  max_velocity_ = 0.15;
  max_angular_velocity_ = 0.5;
  beta_ = 0.2;
  lambda_ = 2.0;
  pnh.param("k1", k1_, k1_);
  pnh.param("k2", k2_, k2_);
  pnh.param("min_velocity", min_velocity_, min_velocity_);
  pnh.param("max_velocity", max_velocity_, max_velocity_);
  pnh.param("max_angular_velocity", max_angular_velocity_,
            max_angular_velocity_);
  pnh.param("beta", beta_, beta_);
  pnh.param("lambda", lambda_, lambda_);

  // TODO: remove these frames
  // frames
  local_frame_ = "base_link";
  pnh.param("base_frame", local_frame_, local_frame_);

  // tolerance
  dist_tolerance_ = 0.01;
  angle_tolerance_ = 0.01;
  pnh.param("xy_tolerance", dist_tolerance_, dist_tolerance_);
  pnh.param("yaw_tolerance", angle_tolerance_, angle_tolerance_);

  ROS_DEBUG_STREAM("lambda is : " << lambda_ << "ns is "
                                  << "~/" + name);
}

bool Controller::setTarget(const geometry_msgs::PoseStamped &target) {
  geometry_msgs::PoseStamped pose = target;
  if (pose.header.frame_id != local_frame_) {
    // Transform target into base frame
    try {
      geometry_msgs::TransformStamped target_to_base_link;
      target_to_base_link =
          tf_buffer_->lookupTransform(local_frame_, // TODO: rosparam
                                      pose.header.frame_id, ros::Time(0));

      ROS_DEBUG_NAMED("graceful controller",
                      "current pose before tf x %3.2f, y %3.2f",
                      pose.pose.position.x, pose.pose.position.y);
      // TODO: check the results
      tf2::doTransform(pose, pose, target_to_base_link);
      ROS_DEBUG_NAMED("graceful controller",
                      "current pose after tf x %3.2f, y %3.2f",
                      pose.pose.position.x, pose.pose.position.y);
    } catch (tf2::TransformException const &ex) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "Couldn't get transform from dock to base_link");
      return false;
    }
  }

  target_ = pose;
  return true;
}

bool Controller::isGoalReached() {
  double x_diff = target_.pose.position.x;
  double y_diff = target_.pose.position.y;
  double dist_error = std::sqrt(x_diff * x_diff + y_diff * y_diff);

  double yaw_error = std::abs(tf2::getYaw(target_.pose.orientation));

  ROS_WARN_STREAM("dist error is " << dist_error << "  yaw error is "
                                   << yaw_error);

  if (dist_error == 0.0 && yaw_error == 0.0)
  {
    ROS_WARN("Control error is zero, is target pose set?");
    return false;
  }

  if (dist_error < dist_tolerance_ && yaw_error < angle_tolerance_)
    return true;
  else
    return false;
}

bool Controller::computeVelocityCommands(
    const geometry_msgs::PoseStamped &current_pose,
    const geometry_msgs::Twist &current_vel, geometry_msgs::Twist &cmd_vel) {
  geometry_msgs::PoseStamped pose = target_;

  // Distance to goal
  double r = std::sqrt(pose.pose.position.x * pose.pose.position.x +
                       pose.pose.position.y * pose.pose.position.y);

  // Orientation base frame relative to r_
  double delta = std::atan2(-pose.pose.position.y, pose.pose.position.x);

  // If within distance tolerance, goal reached
  if (r < 0.01) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.1 * (delta > 0 ? 1 : -1);
    return true;
  }


  // Determine orientation of goal frame relative to r_
  double theta = tf2::getYaw(pose.pose.orientation) + delta;

  // Compute the virtual control
  double a = atan(-k1_ * theta);
  // Compute curvature (k)
  double k = -1.0 / r *
             (k2_ * (delta - a) +
              (1 + k1_ / (1 + ((k1_ * theta) * (k1_ * theta)))) * sin(delta));

  ROS_DEBUG_STREAM("delta is  " << delta << "theta is  " << theta);

  // Compute max_velocity based on curvature
  double v = max_velocity_ / (1 + beta_ * std::pow(fabs(k), lambda_));
  // Limit max velocity based on approaching target (avoids overshoot)
  if (r < 0.75) {
    v = std::max(min_velocity_, std::min(std::min(r, max_velocity_), v));
  } else {
    v = std::min(max_velocity_, std::max(min_velocity_, v));
  }

  // Compute angular velocity
  double w = k * v;
  // Bound angular velocity
  double bounded_w =
      std::min(max_angular_velocity_, std::max(-max_angular_velocity_, w));
  //// Make sure that if we reduce w, we reduce v so that kurvature is still
  /// followed
  // if (w != 0.0)
  //{
  //  v *= (bounded_w/w);
  //}

  // Send command to base
  cmd_vel.linear.x = v;
  cmd_vel.angular.z = bounded_w;

  return true;
}

} // end of namespace
