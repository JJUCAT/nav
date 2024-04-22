#include "navit_bt_nodes/plugins/action/j110/merge_plans_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <navit_bt_nodes/bt_exception.hpp>

namespace navit_bt_nodes {

void MergePlansAction::halt() { }

BT::NodeStatus MergePlansAction::tick()
{
  nav_msgs::Path plan0, plan1;
  size_t knot;
  LoadArg(plan0, plan1, knot);
  ROS_INFO("[BT][%s] plan0 size %lu, plan1 size %lu, knot %lu",
    node_name_,  plan0.poses.size(), plan1.poses.size(), knot);

  if (plan0.poses.empty()) {
    ROS_WARN("[BT][%s] plan0 empty, set plan1 as merged_plan", node_name_);
    setOutput("merged_plan", plan1);
    return BT::NodeStatus::SUCCESS;
  }

  if (knot > plan1.poses.size()-1) {
    ROS_ERROR("[BT][%s] knot error.", node_name_);
    return BT::NodeStatus::FAILURE;
  }

  plan0.poses.insert(plan0.poses.end(), plan1.poses.begin()+knot, plan1.poses.end());
  setOutput("merged_plan", plan0);
  return BT::NodeStatus::SUCCESS;
}

void MergePlansAction::LoadArg(nav_msgs::Path& plan0, nav_msgs::Path& plan1, size_t& knot)
{
  if (!getInput<nav_msgs::Path>("plan0", plan0)) {
    std::string msg("missing arg [plan0]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    // throw(BT_Exception(msg));
  }

  if (!getInput<nav_msgs::Path>("plan1", plan1)) {
    std::string msg("missing arg [plan1]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
  
  if (!getInput<size_t>("knot", knot)) {
    std::string msg("missing arg [knot]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::MergePlansAction>("MergePlansAction");
}

} // namespace navit_bt_nodes
