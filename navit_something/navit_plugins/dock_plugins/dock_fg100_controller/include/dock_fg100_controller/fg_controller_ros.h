#ifndef FG_CONTROLLER_H
#define FG_CONTROLLER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <navit_auto_dock/plugins/approach_dock_controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <dock_fg100_controller/controller.h>
#include <dock_fg100_controller/utils/pose_se2.h>

namespace dock_fg100_controller {
class FgController : public navit_auto_dock::plugins::ApproachDockController {
public:
  FgController() {}

  ~FgController() {}

  void initialize(const std::string name,
                  const std::shared_ptr<tf2_ros::Buffer> &tf);

  bool setTarget(const geometry_msgs::PoseStamped &target_pose);

  bool computeVelocityCommands(const geometry_msgs::PoseStamped &current_pose,
                               const geometry_msgs::Twist &current_vel,
                               geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  PoseSE2 robot_goal_, robot_pose_;
  geometry_msgs::Twist robot_vel_;

  geometry_msgs::Twist last_cmd_vel_;
  ros::Time time_last_cmd_;

  std::string global_frame_, robot_base_frame_;

  // target
  geometry_msgs::PoseStamped global_target_;
  geometry_msgs::PoseStamped current_pose_;

  struct Parameters {
    double xy_goal_tolerance = 0.2;
    double yaw_goal_tolerance = 0.1;
    std::string odom_topic = "odom";
    double controller_frequency = 5;
  } params_;

  bool initialized_;
  bool goal_reached_;

  Controller controller_;
  corbo::TimeSeries::Ptr x_seq_ = std::make_shared<corbo::TimeSeries>();
  corbo::TimeSeries::Ptr u_seq_ = std::make_shared<corbo::TimeSeries>();
};
}

#endif
