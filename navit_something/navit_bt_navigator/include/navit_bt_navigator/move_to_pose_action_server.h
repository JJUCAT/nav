//
// Created by fan on 23-1-30.
//

#ifndef NAVIT_BT_NAVIGATOR_MOVE_TO_POSE_ACTION_SERVER_H
#define NAVIT_BT_NAVIGATOR_MOVE_TO_POSE_ACTION_SERVER_H

#include "navit_bt_navigator/action_server_base.hpp"
#include <navit_msgs/MoveToPoseAction.h>

#include <utility>
#include "navit_bt_navigator/navit_bt_runner.h"

namespace navit_bt_navigator
{
class MoveToPoseActionServer : public ActionServerBase<navit_msgs::MoveToPoseAction>
{
public:
  inline static const std::string ACTION_NAME = "/move_to_pose";

  explicit MoveToPoseActionServer(ros::NodeHandle& nh, std::string xml_string = "")
    : ActionServerBase(nh, ACTION_NAME), default_xml_string_(std::move(xml_string))
  {
  }

protected:
  void executeCB(const GoalTypeConstPtr& goal) override;

protected:
  std::string default_xml_string_;
};
}  // namespace navit_bt_navigator
#endif  // NAVIT_BT_NAVIGATOR_MOVE_TO_POSE_ACTION_SERVER_H
