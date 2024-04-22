#include "navit_bt_navigator/way_points_navigator.hpp"

using namespace BT;
using namespace navit_bt_nodes;


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "way_points_navigator_node");
  ros::NodeHandle bt_node_;
  double server_timeout_;
  double bt_loop_duration_;

  std::string tree_file; 
  tree_file = getParam<std::string>("tree_file", "way_points_navigator.xml");

  std::string tree_full_path_ = ros::package::getPath("navit_bt_navigator") + "/behavior_trees/" + tree_file;
  ROS_INFO("tree_full_path_: [%s]", tree_full_path_.c_str());

  BT::Blackboard::Ptr blackboard_;
  blackboard_ = BT::Blackboard::create();

  // // Put items on the blackboard
  blackboard_->set<ros::NodeHandle>("node", bt_node_);  // NOLINT
  blackboard_->set<double>("server_timeout", server_timeout_);  // NOLINT
  blackboard_->set<double>("bt_loop_duration", bt_loop_duration_);  // NOLINT

  ROS_WARN("Plugins Loading.....");    
  auto bt_ = std::make_unique<navit_bt_nodes::BehaviorTreeEngine>(plugin_lib_names_);
  ROS_WARN("Plugins Loading Finished.");
  auto tree = bt_->createTreeFromFile(tree_full_path_, blackboard_);

  StdCoutLogger logger_cout(tree);
  std::string logger_full_path_ = ros::package::getPath("navit_bt_navigator") + "/logger/" ;
  FileLogger logger_file(tree, (logger_full_path_+"way_points_navigator.fbl").c_str());
  MinitraceLogger logger_minitrace(tree, (logger_full_path_+"way_points_navigator.json").c_str());
  
#ifdef ZMQ_FOUND
  PublisherZMQ publisher_zmq(tree);
#endif
  printTreeRecursively(tree.rootNode());

  // uint16_t publisher_port = 1666, server_port = 1667, max_msg_per_second = 25;
  // bt_->addGrootMonitoring(&tree, publisher_port, server_port, max_msg_per_second);

  // tree.tickRoot();

  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && tree.tickRoot() == NodeStatus::RUNNING)
    ros::Duration(0.01).sleep();

  ros::Duration(30).sleep();
  return 0;
}

