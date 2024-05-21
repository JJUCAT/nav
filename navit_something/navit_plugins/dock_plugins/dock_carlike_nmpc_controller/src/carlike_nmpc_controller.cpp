#include <dock_carlike_nmpc_controller/carlike_nmpc_controller.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dock_carlike_nmpc_controller::CarlikeNmpcController,
                       navit_auto_dock::plugins::ApproachDockController)

namespace dock_carlike_nmpc_controller {
void CarlikeNmpcController::initialize(
    const std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf) {
  tf_buffer_ = tf;
  ros::NodeHandle nh;

  p_[MPC_CONTROLLER_NUM_PARAMETERS] = {0};
  u_[MPC_CONTROLLER_NUM_DECISION_VARIABLES] = {0};
  init_penalty_ = 15.0;
  cache_t_ = mpc_controller_new();
  ROS_DEBUG_STREAM("MPC calculated commands: v_x is "
                   << u_[0] << " steering is " << u_[1]);

  // TODO: rosparams
  control_frame_ = "odom";
  dist_tolerance_ = 0.05;
  angle_tolerance_ = 0.01;
  ROS_INFO_STREAM("NMPC controller initialized");
}

bool CarlikeNmpcController::setTarget(
    const geometry_msgs::PoseStamped &target_pose) {
  geometry_msgs::PoseStamped pose = target_pose;
  if (pose.header.frame_id != control_frame_) {
    // Transform target into control frame
    try {
      geometry_msgs::TransformStamped target_to_odom =
          tf_buffer_->lookupTransform(control_frame_, // TODO: rosparam
                                      pose.header.frame_id, ros::Time(0));

      // TODO: check the results
      tf2::doTransform(pose, pose, target_to_odom);
    } catch (tf2::TransformException const &ex) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "Couldn't get transform from dock to base_link");
      return false;
    }
  }

  target_pose_ = pose;
  return true;
}

bool CarlikeNmpcController::computeVelocityCommands(
    const geometry_msgs::PoseStamped &current_pose,
    const geometry_msgs::Twist &current_vel, geometry_msgs::Twist &cmd_vel) {
  current_pose_ = current_pose;
  geometry_msgs::PoseStamped pose = target_pose_;

  p_[0] = current_pose.pose.position.x;
  p_[1] = current_pose.pose.position.y;
  p_[2] = tf2::getYaw(current_pose.pose.orientation);
  p_[3] = pose.pose.position.x;
  p_[4] = pose.pose.position.y;
  p_[5] = tf2::getYaw(pose.pose.orientation);

  mpc_controllerSolverStatus status =
      mpc_controller_solve(cache_t_, u_, p_, 0, &init_penalty_);

  ROS_DEBUG_STREAM("MPC calculated commands: v_x is "
                   << u_[0] << " steering is " << u_[1] << " solve time is "
                   << status.solve_time_ns / 1000000.0 << " ms ");

  cmd_vel.linear.x = u_[0];
  cmd_vel.angular.z = u_[1];

  return true;
}

bool CarlikeNmpcController::isGoalReached() {
  // TODO: should be in the same frame
  double diff_x = target_pose_.pose.position.x - current_pose_.pose.position.x;
  double diff_y = target_pose_.pose.position.y - current_pose_.pose.position.y;
  double diff_yaw = tf2::getYaw(target_pose_.pose.orientation) -
                    tf2::getYaw(current_pose_.pose.orientation);

  double dist_error = std::sqrt(diff_x * diff_x + diff_y * diff_y);
  double yaw_error = std::abs(diff_yaw);

  if (dist_error < dist_tolerance_ && yaw_error < angle_tolerance_)
    return true;
  else
    return false;
}

CarlikeNmpcController::~CarlikeNmpcController() {
  mpc_controller_free(cache_t_);
}
}
