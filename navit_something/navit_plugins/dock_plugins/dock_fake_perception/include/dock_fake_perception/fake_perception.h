#ifndef DOCK_FAKE_PERCEPTION_H
#define DOCK_FAKE_PERCEPTION_H
#include <navit_auto_dock/plugins/approach_dock_perception.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace dock_fake_perception {
class FakePerception : public navit_auto_dock::plugins::ApproachDockPerception {
public:
  FakePerception() {}

  void initialize(const std::string name,
                  const std::shared_ptr<tf2_ros::Buffer> &tf);

  bool start(const geometry_msgs::PoseStamped &fake_dock_pose);

  bool stop() { return true; }

  bool getPose(geometry_msgs::PoseStamped &pose);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::PoseStamped dock_pose_;

  // params
  std::string tracking_frame_;
  std::string base_frame_;
};
}

#endif
