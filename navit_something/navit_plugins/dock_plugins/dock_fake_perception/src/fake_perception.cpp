#include <dock_fake_perception/fake_perception.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dock_fake_perception::FakePerception,
                       navit_auto_dock::plugins::ApproachDockPerception)

namespace dock_fake_perception {
void FakePerception::initialize(const std::string name,
                                const std::shared_ptr<tf2_ros::Buffer> &tf) {
  tf_buffer_ = tf;

  ros::NodeHandle pnh("~/" + name);
  tracking_frame_ = "odom";
  pnh.param("tracking_frame", tracking_frame_, tracking_frame_);

  ROS_INFO("Fake perception initialized!");
}

bool FakePerception::start(const geometry_msgs::PoseStamped &fake_dock_pose) {
  if (fake_dock_pose.header.frame_id == tracking_frame_) {
    dock_pose_ = fake_dock_pose;
  } else {
    try {
      geometry_msgs::TransformStamped to_tracking_frame_tf;
      to_tracking_frame_tf = tf_buffer_->lookupTransform(
          tracking_frame_, fake_dock_pose.header.frame_id, ros::Time(0));
      dock_pose_.header.frame_id = tracking_frame_;
      tf2::doTransform(fake_dock_pose, dock_pose_, to_tracking_frame_tf);
    } catch (tf2::TransformException const &ex) {
      ROS_WARN("Couldn't transform to the tracking frame %s",
               tracking_frame_.c_str());
      return false;
    }
  }
  return true;
}

bool FakePerception::getPose(geometry_msgs::PoseStamped &pose) {
  pose = dock_pose_;
  return true;
}
}
