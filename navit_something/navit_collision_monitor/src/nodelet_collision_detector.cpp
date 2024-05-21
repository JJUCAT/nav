//
// Created by fan on 23-6-27.
//

#include "navit_collision_monitor/collision_detector.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

namespace nodelet_collision_detector {

class CollisionDetector : public nodelet::Nodelet {
public:
  void onInit() override {
    nh_ = getPrivateNodeHandle();
    node_ = std::make_unique<navit_collision_monitor::CollisionDetector>(nh_);
  }

private:
  ros::NodeHandle nh_;
  std::unique_ptr<navit_collision_monitor::CollisionDetector> node_;
};

} // namespace nodelet_collision_detector
PLUGINLIB_EXPORT_CLASS(nodelet_collision_detector::CollisionDetector,
                       nodelet::Nodelet)
