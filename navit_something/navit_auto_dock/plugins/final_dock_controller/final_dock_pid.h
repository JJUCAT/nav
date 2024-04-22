#ifndef PLUGINS_FINAL_DOCK_PID_H
#define PLUGINS_FINAL_DOCK_PID_H

#include <navit_auto_dock/plugins/final_dock_controller.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

namespace navit_auto_dock {
namespace plugins {
class PidController : public FinalDockController {
public:
  void initialize(std::string name, std::shared_ptr<tf2_ros::Buffer> &tf);

  void setTargetPose(geometry_msgs::PoseStamped &target);

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();

private:
  geometry_msgs::PoseStamped target_, current_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  ros::Subscriber odom_sub_;

  struct {
    double Kp = 0.5;
    double tolerance = 0.01;
    std::string odom_topic_name = "odom";
    double max_v = 0.3;
    double min_v = 0.1;
  } config_;

  double error_;

  bool odom_update_ = false;

  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  inline void clip(double& cmd_v)
  {
      auto sign = cmd_v > 0 ? 1 : -1;
    if ( std::abs(cmd_v) > config_.max_v ) 
        cmd_v = sign * config_.max_v;
    else if (std::abs(cmd_v) < config_.min_v )
        cmd_v = sign * config_.min_v;
  }

};
}
}

#endif
