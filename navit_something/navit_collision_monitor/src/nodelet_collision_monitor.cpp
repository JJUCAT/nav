//
// Created by fan on 23-6-27.
//

#include "navit_collision_monitor/collision_monitor.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

namespace nodelet_collision_monitor {

class CollisionMonitor : public nodelet::Nodelet {
public:
  void onInit() override {
    nh_ = getPrivateNodeHandle();
    node_ = std::make_unique<navit_collision_monitor::CollisionMonitor>(nh_);
  }

private:
  ros::NodeHandle nh_;
  std::unique_ptr<navit_collision_monitor::CollisionMonitor> node_;
};

} // namespace nodelet_collision_detector
PLUGINLIB_EXPORT_CLASS(nodelet_collision_monitor::CollisionMonitor,
                       nodelet::Nodelet)
