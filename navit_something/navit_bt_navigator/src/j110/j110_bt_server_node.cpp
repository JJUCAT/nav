#include "navit_bt_navigator/j110/auto_dock_server.hpp"
#include "navit_bt_navigator/j110/move_area_a2b_server.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "j110_bt_navigator_node");
  ros::NodeHandle nh;
  std::string server_type;
  
  // server_type 支持多个类型，从xml file中读取
  nh.getParam("server_type", server_type);

  if (server_type == "auto_dock") {
    navit_bt_navigator::AutoDockActionServer server(nh);
  } else {
    ROS_ERROR("server_type %s is not supported!", server_type.c_str());
  }

  ros::spin();
  return 0;
}