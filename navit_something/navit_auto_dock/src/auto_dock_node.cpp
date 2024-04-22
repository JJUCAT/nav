#include <navit_auto_dock/approach_dock.h>
#include <navit_auto_dock/final_dock.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "auto_dock");
  ros::NodeHandle nh("~");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf_listener_(*tf_buffer);

  navit_auto_dock::ApproachDock approach_dock("/auto_dock/approach_dock", tf_buffer,nh);
  navit_auto_dock::FinalDock final_dock("/auto_dock/final_dock", tf_buffer, nh);
  ros::spin();
  return 0;
}
