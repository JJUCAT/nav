#ifndef NAVIT_BT_NAVIGATOR_BACK_TO_REFERENCE_LINE_ACTION_SERVER_H
#define NAVIT_BT_NAVIGATOR_BACK_TO_REFERENCE_LINE_ACTION_SERVER_H

#include "navit_bt_navigator/action_server_base.hpp"
#include <navit_msgs/BackToReferenceLineAction.h>
#include <nav_msgs/Path.h>
#include <utility>
#include "navit_bt_navigator/navit_bt_runner.h"

namespace navit_bt_navigator
{
class BackToReferenceLineActionServer : public ActionServerBase<navit_msgs::BackToReferenceLineAction>
{
public:
   inline static const std::string action_name = "/back_to_reference_line";

  explicit BackToReferenceLineActionServer(ros::NodeHandle& nh, std::string xml_string = "")
    : ActionServerBase(nh, action_name), default_xml_string_(std::move(xml_string))
  {
  }

protected:
  void executeCB(const GoalTypeConstPtr& goal) override;

protected:
  std::string default_xml_string_;
};
}  // namespace navit_bt_navigator
#endif  // NAVIT_BT_NAVIGATOR_MOVE_TO_POSE_ACTION_SERVER_H
