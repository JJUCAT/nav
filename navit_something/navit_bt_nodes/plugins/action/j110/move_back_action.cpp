#include "navit_bt_nodes/plugins/action/j110/move_back_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "navit_msgs/BackUpResult.h"
#include <navit_bt_nodes/bt_exception.hpp>
#include <functional> 

namespace navit_bt_nodes {

void MoveBackAction::halt()
{
  if (status() == BT::NodeStatus::RUNNING)
    client_->cancelGoal();

  setStatus(BT::NodeStatus::IDLE);
}

BT::NodeStatus MoveBackAction::tick()
{
  double back, speed;
  LoadArg(back, speed);

  BT::NodeStatus node_state = BT::NodeStatus::RUNNING;

  if (state_ == kIdle) {
    ROS_INFO("[BT][%s] wait for [backup] server.", node_name_);
    client_->waitForServer();    
    ROS_INFO("[BT][%s] ready to move back, back %.3f, speed %.3f.", node_name_, back, speed);
    state_ = kRunning;
    navit_msgs::BackUpGoal goal;
    goal.backup_distance = back;
    goal.speed = speed;
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

void MoveBackAction::LoadArg(double& back, double& speed)
{
  if (!getInput<double>("back", back)) {
    std::string msg("missing arg [back]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("speed", speed)) {
    std::string msg("missing arg [speed]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }
}

void MoveBackAction::ResultCallback(
  const actionlib::SimpleClientGoalState &state, const navit_msgs::BackUpResultConstPtr &result)
{
  if (state.state_ == state.SUCCEEDED) {
    ROS_INFO("[BT][%s] succeed.", node_name_);
    state_ = kSucceed;
  } else {
    ROS_WARN("[BT][%s] failed.", node_name_);
    state_ = kFailed;
  }
}

void MoveBackAction::ActiveCallback()
{
  ROS_INFO("[BT][%s] active...", node_name_);
}

void MoveBackAction::FeedbackCallback(const navit_msgs::BackUpFeedbackConstPtr &feedback)
{
  // ROS_INFO_STREAM_THROTTLE
  ROS_INFO_THROTTLE(2, "[BT][%s] feadback, distance traveled %.3f.", node_name_, feedback->distance_traveled);
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::MoveBackAction>("MoveBackAction");
}

} // namespace navit_bt_nodes
