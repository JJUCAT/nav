#include "navit_bt_nodes/plugins/action/j110/select_goal_on_plan_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/duration.h"
#include <navit_bt_nodes/bt_exception.hpp>

namespace navit_bt_nodes {

void SelectGoalOnPlanAction::halt()
{
  last_plan_seq_ = std::numeric_limits<size_t>::max();
  distance_ = 0;
  start_time_ = ros::Time::now();
}

BT::NodeStatus SelectGoalOnPlanAction::tick()
{
  nav_msgs::Path ref_plan;
  double distance, jump, timeout;
  size_t start;
  LoadArg(ref_plan, distance, jump, timeout, start);

  bool new_plan = IsNewPlan(ref_plan);
  if (IsTimeout(timeout)) {
    return BT::NodeStatus::FAILURE;
  }

  double step = new_plan ? 0.0 : jump;
  ROS_INFO("[BT][%s] distance %.3f, step %.3f, timeout %.3f, start %lu.",
    node_name_, distance, step, timeout, start);
  size_t goal_index = Select(ref_plan, distance, step, start);
  geometry_msgs::PoseStamped goal;
  goal = ref_plan.poses.at(goal_index);
  ROS_INFO("[BT][%s] goal index %lu, goal [%f,%f]",
    node_name_, goal_index, goal.pose.position.x, goal.pose.position.y);

  setOutput("goal", goal);
  setOutput("goal_index", goal_index);
  setOutput("start", goal_index);
  return BT::NodeStatus::SUCCESS;
}

void SelectGoalOnPlanAction::LoadArg(nav_msgs::Path& ref_plan,
  double& distance, double& jump, double& timeout, size_t& start)
{
  if (!getInput<nav_msgs::Path>("ref_plan", ref_plan)) {
    std::string msg("missing arg [ref_plan]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("distance", distance)) {
    std::string msg("missing arg [distance]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
  
  if (!getInput<double>("jump", jump)) {
    std::string msg("missing arg [jump]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("timeout", timeout)) {
    std::string msg("missing arg [timeout]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<size_t>("start", start)) {
    std::string msg("missing arg [start]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
}

bool SelectGoalOnPlanAction::IsNewPlan(const nav_msgs::Path& ref_plan)
{
  if (ref_plan.header.seq != last_plan_seq_) {
    ROS_INFO("[BT][%s] new plan (%lu), seq %u",
      node_name_, ref_plan.poses.size(), ref_plan.header.seq);
    last_plan_seq_ = ref_plan.header.seq;
    start_time_ = ros::Time::now();
    distance_ = 0.0;
    return true;
  }
  return false;
}

bool SelectGoalOnPlanAction::IsTimeout(const double timeout)
{
  ros::Duration to(timeout);
  if (ros::Time::now() - start_time_ > to) {
    ROS_ERROR("[BT][%s] overtime %.3f seconds, timeout !", node_name_, timeout);
    return true;
  }
  return false;
}

size_t SelectGoalOnPlanAction::Select(
  const nav_msgs::Path& ref_plan,const double distance, const double jump, const size_t start)
{
  auto& poses = ref_plan.poses;
  if (poses.size() == 1) return 0;

  double d, check = 0;
  size_t i = start;
  for (size_t j = i; ; j = i, i ++) {
    if (i == poses.size()) { i = j = 0; }
    d = std::hypot(poses.at(i).pose.position.y - poses.at(j).pose.position.y,
                   poses.at(i).pose.position.x - poses.at(j).pose.position.x);
    distance_+= d;
    if (distance_ > distance) {
      distance_ = 0;
      i = j = 0;
    }
    check += d;
    if (check >= jump) break;
  }
  return i;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::SelectGoalOnPlanAction>("SelectGoalOnPlanAction");
}

} // namespace navit_bt_nodes
