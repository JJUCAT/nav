#ifndef DIFF_PUREPURSUIT_PID_CONTROLLER_H
#define DIFF_PUREPURSUIT_PID_CONTROLLER_H

#include <navit_auto_dock/plugins/approach_dock_controller.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <kdl/frames.hpp>

namespace dock_diff_purepursuit_pid_controller {

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
  geometry_msgs::PoseStamped target_, current_pose_;

  std::string global_frame_;
  std::string local_frame_;

  /*
   * Parameters for controller
   */
  double half_length_robot_;//车身半长，哟过来当作车身长
  double ld_; //前视距离Ld=ld_+车距离直线的横向距离，这个距离是为了防止ld过短造成的震荡
  double k_control_; //前视距离与车速度的比值，用来控制转向角度

  /*
   * Parameters for velocity limits
   */
  double min_velocity_ ;
  double max_velocity_ ;
  double max_angular_velocity_ ;

  double beta_,lambda_;


  /*
   * Tolerance
   */
  double dist_tolerance_, angle_tolerance_;
};

} // end of namespace
#endif
