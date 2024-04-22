#include "navit_bt_nodes/plugins/action/j110/excape_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "navit_msgs/BackUpResult.h"
#include <navit_bt_nodes/bt_exception.hpp>
#include <functional> 

namespace navit_bt_nodes {

void ExcapeAction::halt()
{
  if (status() == BT::NodeStatus::RUNNING)
    client_->cancelGoal();

  setStatus(BT::NodeStatus::IDLE);
}

BT::NodeStatus ExcapeAction::tick()
{
  double timeout, free, v, w, r;
  LoadArg(timeout, free, v, w, r);

  BT::NodeStatus node_state = BT::NodeStatus::RUNNING;

  if (state_ == kIdle) {
    ROS_INFO("[BT][%s] wait for [excape] server.", node_name_);
    client_->waitForServer();    
    ROS_INFO("[BT][%s] ready to excape, timeout %.3f, free %.3f.", node_name_, timeout, free);
    state_ = kRunning;
    navit_msgs::ExcapeGoal goal;
    goal.timeout = timeout;
    goal.free = free;
    goal.v = v;
    goal.w = w;
    goal.r = r;
    client_->sendGoal(goal,
    [this](auto& _1, auto& _2) {ResultCallback(_1, _2);},
    [this]() {ActiveCallback();},
    [this](auto& _1) {FeedbackCallback(_1);});
  } else if ( state_ != kRunning ){
    node_state = state_ == kSucceed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    state_ = kIdle;
  }

  return node_state;
}

void ExcapeAction::LoadArg(double& timeout, double& free, double& v, double& w, double& r)
{
  if (!getInput<double>("timeout", timeout)) {
    std::string msg("missing arg [timeout]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("free", free)) {
    std::string msg("missing arg [free]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("v", v)) {
    std::string msg("missing arg [v]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("w", w)) {
    std::string msg("missing arg [w]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("r", r)) {
    std::string msg("missing arg [r]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }
}

void ExcapeAction::ResultCallback(
  const actionlib::SimpleClientGoalState &state, const navit_msgs::ExcapeResultConstPtr &result)
{
  if (state.state_ == state.SUCCEEDED) {
    ROS_INFO("[BT][%s] succeed.", node_name_);
    state_ = kSucceed;
  } else {
    ROS_WARN("[BT][%s] failed.", node_name_);
    state_ = kFailed;
  }
}

void ExcapeAction::ActiveCallback()
{
  ROS_INFO("[BT][%s] active...", node_name_);
}

void ExcapeAction::FeedbackCallback(const navit_msgs::ExcapeFeedbackConstPtr &feedback)
{
  // ROS_INFO_STREAM_THROTTLE
  ROS_INFO_THROTTLE(2, "[BT][%s] feadback, time lapses %.3f.", node_name_, feedback->elapsed_time);
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::ExcapeAction>("ExcapeAction");
}

} // namespace navit_bt_nodes
