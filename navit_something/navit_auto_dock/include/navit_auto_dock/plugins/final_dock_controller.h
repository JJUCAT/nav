#ifndef PLUGINS_FINAL_DOCK_CONTROLLER_H
#define PLUGINS_FINAL_DOCK_CONTROLLER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

namespace navit_auto_dock {
namespace plugins {
class FinalDockController {
public:
    using Ptr = boost::shared_ptr<FinalDockController>;
  virtual ~FinalDockController() {}

  virtual void initialize(std::string name,
                          std::shared_ptr<tf2_ros::Buffer> &) = 0;

  // set point of the controller
  virtual void setTargetPose(geometry_msgs::PoseStamped &target) = 0;

  virtual bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) = 0;

  virtual bool isGoalReached() = 0;
};
}
}

#endif
