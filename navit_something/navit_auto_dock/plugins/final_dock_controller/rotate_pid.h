#ifndef PLUGINS_ROTATE_PID_H
#define PLUGINS_ROTATE_PID_H

#include <navit_auto_dock/plugins/rotate_controller.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

namespace navit_auto_dock {
namespace plugins {
class RotatePidController : public RotateController {
public:
  void initialize(std::string name, std::shared_ptr<tf2_ros::Buffer> &tf);

  bool rotateToTarget(const geometry_msgs::PoseStamped &target);

private:
  geometry_msgs::PoseStamped target_;

  geometry_msgs::Twist cmd_vel_;

  ros::NodeHandle nh;
  ros::Publisher cmd_pub_;
  ros::Subscriber odom_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  double current_yaw_;

  struct config 
  {
    double rotate_timeout = 30.0;
    double Kp = 0.1;
    double max_vel = 0.5;
    double min_vel = 0.05;
    double control_frequency = 50.0;
    double yaw_tolerance = 0.01;
  } config_;

  inline double getYawError() {
    double error = tf2::getYaw(target_.pose.orientation) - current_yaw_;
    return error;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
};
}
}

#endif
