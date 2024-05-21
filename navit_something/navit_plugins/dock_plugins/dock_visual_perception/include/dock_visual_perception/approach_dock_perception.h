#ifndef NAVIT_AUTO_DOCK__APPROACHE_DOCK_PERCEPTION_H
#define NAVIT_AUTO_DOCK__APPROACHE_DOCK_PERCEPTION_H

#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

namespace navit_auto_dock {
namespace plugins {
class ApproachDockPerception {
public:
  using Ptr = boost::shared_ptr<ApproachDockPerception>;

  virtual ~ApproachDockPerception() {}

  virtual void initialize(const std::string name,const std::shared_ptr<tf2_ros::Buffer> &) = 0;
                          
  virtual bool start(const geometry_msgs::PoseStamped &target_pose) = 0;

  virtual bool getPose(geometry_msgs::PoseStamped &dock_pose) = 0;

  virtual bool stop() = 0;
};
}
}

#endif /* NAVIT_AUTO_DOCK__APPROACHE_DOCK_PERCEPTION_H*/