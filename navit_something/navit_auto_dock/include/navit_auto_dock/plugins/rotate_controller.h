#ifndef PLUGINS_ROTATE_CONTROLLER_H
#define PLUGINS_ROTATE_CONTROLLER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

namespace navit_auto_dock {
namespace plugins {
class RotateController {
public:
    using Ptr = boost::shared_ptr<RotateController>;
  virtual ~RotateController() {}

  virtual void initialize(std::string name,
                          std::shared_ptr<tf2_ros::Buffer> &) = 0;

  // set point of the controller
  virtual void setTargetPose(geometry_msgs::PoseStamped &target) {
    tf2::Quaternion q_rot, q_orig, q_new;

    q_rot.setRPY(0, 0, M_PI);

    tf2::convert(target.pose.orientation, q_orig);

    q_new = q_rot * q_orig;

    tf2::convert(q_new, target.pose.orientation);
  }

  // blocking until the goal is reached
  virtual bool rotateToTarget(const geometry_msgs::PoseStamped &target) = 0;
};
}
}

#endif
