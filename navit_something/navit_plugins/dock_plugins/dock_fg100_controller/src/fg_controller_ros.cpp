#include <dock_fg100_controller/fg_controller_ros.h>

#include <dock_fg100_controller/utils/math_utils.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dock_fg100_controller::FgController,
                       navit_auto_dock::plugins::ApproachDockController)

namespace dock_fg100_controller {

void FgController::initialize(const std::string name,
                              const std::shared_ptr<tf2_ros::Buffer> &tf) {
  // create Node Handle with name of plugin (as used in move_base for loading)
  ros::NodeHandle nh("~/" + name);

  nh.param("xy_goal_tolerance", params_.xy_goal_tolerance,
           params_.xy_goal_tolerance);
  nh.param("yaw_goal_tolerance", params_.yaw_goal_tolerance,
           params_.yaw_goal_tolerance);

  // init other variables
  tf_buffer_ = tf;

  global_frame_ = "odom";
  robot_base_frame_ = "base_link";

  // create the planner instance
  if (!controller_.configure(nh)) {
    ROS_ERROR("Controller configuration failed.");
    return;
  }

  // init the odom helper to receive the robot's velocity from odom messages

  // setup callback for custom obstacles
  // additional move base params
  ros::NodeHandle pnh("~");
  pnh.param("controller_frequency", params_.controller_frequency,
            params_.controller_frequency);

  // set initialized flag
  initialized_ = true;
  goal_reached_ = false;

  ROS_DEBUG("dock_fg100_controller plugin initialized.");
}

bool FgController::setTarget(const geometry_msgs::PoseStamped &orig_target) {
  // check if plugin is initialized
  if (!initialized_) {
    ROS_ERROR("dock_fg100_controller has not been initialized, please call "
              "initialize() before using this controller");
    return false;
  }

  if (orig_target.header.frame_id != global_frame_) {
    try {
      geometry_msgs::TransformStamped tf_pose_to_odom =
          tf_buffer_->lookupTransform(
              global_frame_, orig_target.header.frame_id, ros::Time(0));
      global_target_.header.frame_id = global_frame_;
      tf2::doTransform(orig_target, global_target_, tf_pose_to_odom);
    } catch (const tf2::TransformException &ex) {
      ROS_FATAL("Failed to transform to global frame %s", ex.what());
      exit(1);
    }
  } else {
    global_target_ = orig_target;
  }

  return true;
}

bool FgController::computeVelocityCommands(
    const geometry_msgs::PoseStamped &current_pose,
    const geometry_msgs::Twist &current_vel, geometry_msgs::Twist &cmd_vel) {
  // check if plugin initialized
  if (!initialized_) {
    ROS_ERROR("dock_fg100_controller has not been initialized, please call "
              "initialize() before using this planner");
    return false;
  }

  cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;

  // Get robot pose and vel
  robot_pose_.x() = current_pose.pose.position.x;
  robot_pose_.y() = current_pose.pose.position.y;
  robot_pose_.theta() = tf2::getYaw(current_pose.pose.orientation);

  robot_vel_ = current_vel;
  current_pose_ = current_pose;

  // check if global goal is reached
  double dx = global_target_.pose.position.x - robot_pose_.x();
  double dy = global_target_.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta(
      tf2::getYaw(global_target_.pose.orientation) - robot_pose_.theta());
  double dist_error = std::abs(std::sqrt(dx * dx + dy * dy));
  double yaw_error = std::abs(delta_orient);
  ROS_DEBUG("MPC control error is %f %f ", dist_error, yaw_error);

  if (dist_error < params_.xy_goal_tolerance &&
      yaw_error < params_.yaw_goal_tolerance) {
    goal_reached_ = true;
    return true;
  }

  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = global_target_.pose.position.x;
  robot_goal_.y() = global_target_.pose.position.y;
  robot_goal_.theta() = tf2::getYaw(global_target_.pose.orientation);

  std::vector<geometry_msgs::PoseStamped> plan;
  plan.push_back(current_pose_);
  plan.push_back(global_target_);

  ros::Time t = ros::Time::now();
  double dt = 1.0 / params_.controller_frequency;

  // set previous control value for control deviation bounds
  if (u_seq_ && !u_seq_->isEmpty())
    controller_.getOptimalControlProblem()->setPreviousControlInput(
        u_seq_->getValuesMap(0), dt);

  bool success = false;

  success = controller_.step(robot_pose_, robot_goal_, robot_vel_, dt, t,
                             u_seq_, x_seq_);

  if (!success) {
    controller_.reset(); // force reinitialization for next time
    ROS_WARN("dock_fg100_controller was not able to obtain a local plan for "
             "the current setting.");
    last_cmd_vel_ = cmd_vel;
    return false;
  }

  // Get the velocity command for this sampling interval
  // TODO(roesmann): we might also command more than just the imminent action,
  // e.g. in a separate thread, until a new command is available
  if (!u_seq_ ||
      !controller_.getRobotDynamics()->getTwistFromControl(
          u_seq_->getValuesMap(0), cmd_vel)) {
    controller_.reset();
    ROS_WARN("FgController: velocity command invalid. Resetting controller...");
    last_cmd_vel_ = cmd_vel;
    return false;
  }

  // store last command (for recovery analysis etc.)
  last_cmd_vel_ = cmd_vel;
  time_last_cmd_ = ros::Time::now();

  return true;
}

bool FgController::isGoalReached() {
  if (goal_reached_) {
    ROS_DEBUG("Goal reached!!");
    goal_reached_ = false;
    return true;
  }
  return false;
}
}
