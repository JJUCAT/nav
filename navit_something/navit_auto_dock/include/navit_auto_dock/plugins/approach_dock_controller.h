#ifndef NAVIT_AUTO_DOCK__APPROACH_DOCK_CONTROLLER_H
#define NAVIT_AUTO_DOCK__APPROACH_DOCK_CONTROLLER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>

namespace navit_auto_dock {
namespace plugins {
class ApproachDockController {
public:
  using Ptr = boost::shared_ptr<ApproachDockController>;

  virtual ~ApproachDockController() {}

  virtual void initialize(const std::string name,
                          const std::shared_ptr<tf2_ros::Buffer> &tf) = 0;

  virtual bool setTarget(const geometry_msgs::PoseStamped &target_pose) = 0;

  virtual bool
  computeVelocityCommands(const geometry_msgs::PoseStamped &current_pose,
                          const geometry_msgs::Twist &current_vel,
                          geometry_msgs::Twist &cmd_vel) = 0;

  virtual bool isGoalReached() = 0;
};
}
}

#endif /* ifndef NAVIT_AUTO_DOCK__APPROACH_DOCK_CONTROLLER_H*/
