#include <navit_auto_dock/dock_calibration.h>
#include <tf2_ros/transform_listener.h>
// #include <navit_auto_dock/approach_dock.h>
// #include <navit_auto_dock/final_dock.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "auto_dock");
  ros::NodeHandle nh("~");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf_listener_(*tf_buffer);

  //TODO: collision checker params from ros param
  // std::string costmap_topic_name = "/navit_controller_node/controller_costmap/costmap";
  // std::string footprint_topic_name = "/navit_controller_node/controller_costmap/footprint";
  // std::string odom_frame = "odom";
  // std::string base_frame = "base_link";
  // nh.param("costmap_topic_name", costmap_topic_name, costmap_topic_name);
  // nh.param("footprint_topic_name", footprint_topic_name, footprint_topic_name);
  // nh.param("odom_frame_id", odom_frame, odom_frame);
  // nh.param("base_frame_id", base_frame, base_frame);

  // auto costmap_sub = std::make_shared<navit_collision_checker::CostmapSub>(nh, costmap_topic_name);
  // auto footprint_sub = std::make_shared<navit_collision_checker::FootprintSub>(nh, footprint_topic_name);

  // auto collision_checker = std::make_shared<navit_collision_checker::CollisionChecker>(costmap_sub, 
  //                                                                                   footprint_sub, 
  //                                                                                   tf_buffer,
  //                                                                                   "collision_checker",
  //                                                                                   odom_frame,
  //                                                                                   base_frame); 

  //navit_auto_dock::ApproachDock approach_dock("/auto_dock/approach_dock", tf_buffer, collision_checker, nh);
  // navit_auto_dock::ApproachDock approach_dock("/auto_dock/approach_dock", tf_buffer,nh);
  // navit_auto_dock::FinalDock final_dock("/auto_dock/final_dock", tf_buffer, collision_checker, nh);
 navit_auto_dock::CalibrateDock calibrateDock("/auto_dock/approach_dock", tf_buffer, nh);




  ros::spin();
  return 0;
}
