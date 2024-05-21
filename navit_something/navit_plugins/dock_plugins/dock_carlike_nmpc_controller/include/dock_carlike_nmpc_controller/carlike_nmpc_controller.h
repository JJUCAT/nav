#ifndef CARLIKE_NMPC_CONTROLLER_H
#define CARLIKE_NMPC_CONTROLLER_H

#include <navit_auto_dock/plugins/approach_dock_controller.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <dock_carlike_nmpc_controller/solver/mpc_controller_bindings.hpp>

namespace dock_carlike_nmpc_controller {
class CarlikeNmpcController
    : public navit_auto_dock::plugins::ApproachDockController {
public:
  CarlikeNmpcController() {}

  ~CarlikeNmpcController();

  void initialize(const std::string name,
                  const std::shared_ptr<tf2_ros::Buffer> &tf);

  bool setTarget(const geometry_msgs::PoseStamped &target_pose);

  bool computeVelocityCommands(const geometry_msgs::PoseStamped &current_pose,
                               const geometry_msgs::Twist &current_vel,
                               geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  ros::Subscriber odom_sub_;

  // control params
  double p_[MPC_CONTROLLER_NUM_PARAMETERS];
  double u_[MPC_CONTROLLER_NUM_DECISION_VARIABLES];
  double init_penalty_;
  double x_ref[3];
  mpc_controllerCache *cache_t_;

  // poses
  geometry_msgs::PoseStamped current_pose_, target_pose_;

  // params
  std::string control_frame_;
  double dist_tolerance_, angle_tolerance_;
};
}

#endif
