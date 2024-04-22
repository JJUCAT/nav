//
// Created by fan on 23-1-30.
//

#ifndef NAVIT_BT_NAVIGATOR_ACTION_SERVER_BASE_HPP
#define NAVIT_BT_NAVIGATOR_ACTION_SERVER_BASE_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

namespace navit_bt_navigator
{
template <typename ActionT>
class ActionServerBase
{
public:
  using ActionType = ActionT;
  using ActionServerType = actionlib::SimpleActionServer<ActionType>;

  using GoalType = typename ActionType::_action_goal_type::_goal_type;
  using GoalTypeConstPtr = typename GoalType::ConstPtr;
  using FeedbackType = typename ActionType::_action_feedback_type::_feedback_type;
  using FeedbackTypeConstPtr = typename FeedbackType::ConstPtr;
  using ResultType = typename ActionType::_action_result_type::_result_type;
  using ResultTypeConstPtr = typename ResultType::ConstPtr;

public:
  ActionServerBase(ros::NodeHandle& nh, const std::string& action_name)
    : nh_(nh)
    , action_name_(action_name)
    , as_(nh_, action_name_, boost::bind(&ActionServerBase::executeCB, this, _1), false)
  {
    as_.start();
  }

protected:
  virtual void executeCB(const GoalTypeConstPtr& goal) = 0;

  ros::NodeHandle nh_;
  std::string action_name_;
  ActionServerType as_;

  // create messages that are used to published feedback/result
  FeedbackType feedback_;
  ResultType result_;
};
}  // namespace navit_bt_navigator

#endif  // NAVIT_BT_NAVIGATOR_ACTION_SERVER_BASE_HPP
