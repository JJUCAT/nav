#include "navit_bt_nodes/plugins/action/j110/select_start_on_plan_action.hpp"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <navit_bt_nodes/bt_exception.hpp>

namespace navit_bt_nodes {

void SelectStartOnPlanAction::halt()
{

}

BT::NodeStatus SelectStartOnPlanAction::tick()
{
  nav_msgs::Path ref_plan;
  std::vector<uint32_t> collised_indexes;
  double jump;
  LoadArg(ref_plan, collised_indexes, jump);
  ROS_INFO("[BT][%s] ref plan size %lu, collised_indexes size %lu, jump %f",
    node_name_,  ref_plan.poses.size(), collised_indexes.size(), jump);

  size_t start = 0;
  if (!collised_indexes.empty()) {
    size_t i = static_cast<size_t>(collised_indexes.back());
    if (i >= ref_plan.poses.size()) {
      ROS_ERROR("[BT][%s] final collised index >= ref plan size, error !", node_name_);
      return BT::NodeStatus::FAILURE;
    }
    double dist = 0.0f;
    auto& poses = ref_plan.poses;
    for (auto j = i; i < poses.size(); j = i, i++) {
      dist += std::hypot(poses.at(i).pose.position.y - poses.at(j).pose.position.y,
                         poses.at(i).pose.position.x - poses.at(j).pose.position.x);
      if (dist >= jump) break;
    }
    start = i;
  }

  setOutput("start", start);
  return BT::NodeStatus::SUCCESS;
}

void SelectStartOnPlanAction::LoadArg(nav_msgs::Path& ref_plan, std::vector<uint32_t>& collised_indexes, double& jump)
{
  if (!getInput<nav_msgs::Path>("ref_plan", ref_plan)) {
    std::string msg("missing arg [ref_plan]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<std::vector<uint32_t>>("collised_indexes", collised_indexes)) {
    std::string msg("missing arg [collised_indexes]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
  
  if (!getInput<double>("jump", jump)) {
    std::string msg("missing arg [jump]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
}


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::SelectStartOnPlanAction>("SelectStartOnPlanAction");
}

} // namespace navit_bt_nodes
