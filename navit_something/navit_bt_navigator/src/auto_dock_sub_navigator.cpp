#include "navit_bt_nodes/behavior_tree_engine.hpp"

#include <ros/ros.h>
#include <ros/package.h>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

const std::vector<std::string> plugin_lib_names_ = {
    "navit_bt_nodes_approach_dock_action",
    "navit_bt_nodes_final_dock_action"    
  };

template<typename T>
T getParam(const std::string& name,const T& defaultValue) //This name must be namespace+parameter_name
{
    T v;
    if(ros::param::get(name,v)) //get parameter by name depend on ROS.
    {
        ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
        return v;
    }
    else
        ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
    return defaultValue; //if the parameter haven't been set,it's value will return defaultValue.
};


using namespace BT;
using namespace navit_bt_nodes;


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "auto_dock_sub_navigator");
  std::string tree_file; 
  tree_file = getParam<std::string>("tree_file", "auto_dock_subtree.xml");

  std::string tree_file_name = ros::package::getPath("navit_bt_navigator") + "/behavior_trees/"+ tree_file;
  ROS_INFO("tree_file_name: [%s]", tree_file_name.c_str());

  BT::Blackboard::Ptr blackboard_;
  blackboard_ = BT::Blackboard::create();
  ros::NodeHandle client_node_;
  double server_timeout;
  double bt_loop_duration;

  // // Put items on the blackboard
  blackboard_->set<ros::NodeHandle>("node", client_node_);  // NOLINT
  blackboard_->set<double>("server_timeout", server_timeout);  // NOLINT
  blackboard_->set<double>("bt_loop_duration", bt_loop_duration);  // NOLINT

  ROS_WARN("Plugins Loading.....");    
  auto bt_ = std::make_unique<navit_bt_nodes::BehaviorTreeEngine>(plugin_lib_names_);
  ROS_WARN("Plugins Loading Finished.");    
  // auto tree = bt_->createTreeFromText(xml_text, blackboard_);
  auto tree = bt_->createTreeFromFile(tree_file_name, blackboard_);

  StdCoutLogger logger_cout(tree);
  FileLogger logger_file(tree, "auto_dock_subtree.fbl");
  MinitraceLogger logger_minitrace(tree, "auto_dock_subtree.json");
#ifdef ZMQ_FOUND
  PublisherZMQ publisher_zmq(tree);
#endif
  printTreeRecursively(tree.rootNode());

  // uint16_t publisher_port = 1666, server_port = 1667, max_msg_per_second = 25;
  // bt_->addGrootMonitoring(&tree, publisher_port, server_port, max_msg_per_second);

  // NodeStatus status = tree.tickRoot();

  // NodeStatus status = NodeStatus::RUNNING;
  // while (status == NodeStatus::RUNNING)
  // {
  //   status = tree.tickRoot();

  //   ros::Duration(0.05).sleep();
  // } 


  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while(ros::ok() && tree.tickRoot() == NodeStatus::RUNNING)
  {
      ros::Duration(0.01).sleep();
  }
  
  return 0;
}

