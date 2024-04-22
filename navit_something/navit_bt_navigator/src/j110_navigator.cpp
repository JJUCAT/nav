#include "navit_bt_navigator/j110_navigator.hpp"
#include <ros/ros.h>
#include <navit_common/log.h>
#include <sensor_msgs/BatteryState.h>

using namespace BT;
using namespace navit_bt_nodes;



namespace j110_bt_navigator {
class J110Navigator
{

public:
  J110Navigator() {};
  ~J110Navigator() {};
  void run();

private:
  void loadNewBehaviorTree(const std::string& new_tree_file);
  void batteryStatusCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

  ros::NodeHandle nh_;
  std::unique_ptr<BehaviorTreeEngine> bt_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  float battery_voltage_;
  int tree_tick_count_;
};

void J110Navigator::run() {
  NAVIT_ROS_INFO_STREAM("J110Navigator::run()");

  std::string tree_file;
  nh_.param("tree_file", tree_file, std::string("default_tree.xml"));
  std::string tree_full_path_ = ros::package::getPath("navit_bt_navigator") + "/behavior_trees/" + tree_file;
  
  NAVIT_ROS_INFO_STREAM("Plugins Loading...");
  bt_ = std::make_unique<navit_bt_nodes::BehaviorTreeEngine>(plugin_lib_names_);
  NAVIT_ROS_INFO_STREAM("Plugins Load finished.");
  
  loadNewBehaviorTree("default_tree.xml");

  blackboard_ = BT::Blackboard::create();

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    tree_.tickRoot();
    ros::spinOnce();
    loop_rate.sleep();
  }
}


void J110Navigator::loadNewBehaviorTree(const std::string& new_tree_file) {
  std::string new_tree_full_path = ros::package::getPath("navit_bt_navigator") + "/behavior_trees/" + new_tree_file;
  tree_ = bt_->createTreeFromFile(new_tree_full_path, blackboard_);
}

void J110Navigator::batteryStatusCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
  battery_voltage_ = msg->voltage;
}  

} // namespace j110_bt_navigator

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "j110_bt_navigator_node");
  j110_bt_navigator::J110Navigator j110_navigator;
  j110_navigator.run();
  return 0;
}