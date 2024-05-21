#ifndef DIFF_GRACEFUL_CONTROLLER_H
#define DIFF_GRACEFUL_CONTROLLER_H

#include <navit_auto_dock/plugins/approach_dock_controller.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace dock_diff_graceful_controller {

class Controller : public navit_auto_dock::plugins::ApproachDockController {
public:
  Controller() {}

  void initialize(const std::string name,
                  const std::shared_ptr<tf2_ros::Buffer> &tf);

  bool setTarget(const geometry_msgs::PoseStamped &target);
  /**
   * @brief Implements something loosely based on "A Smooth Control Law for
   * Graceful Motion of Differential Wheeled Mobile Robots in 2D Environments"
   * by Park and Kuipers, ICRA 2011
   * @returns true if base has reached goal.
   */
  bool computeVelocityCommands(const geometry_msgs::PoseStamped &current_pose,
                               const geometry_msgs::Twist &current_vel,
                               geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  bool goal_reached_;

  std::string global_frame_;
  std::string local_frame_;
  /*
   * Parameters for controller
   */
  double k1_; // ratio in change of theta to rate of change in r
  double k2_; // speed at which we converge to slow system
  double min_velocity_;
  double max_velocity_;
  double max_angular_velocity_;
  double beta_;   // how fast velocity drops as k increases
  double lambda_; // ??
  double dist_;   // used to create the tracking line

  geometry_msgs::PoseStamped target_, current_pose_;

  /*
   * Tolerance
   */
  double dist_tolerance_, angle_tolerance_;
};

} // end of namespace
#endif
