#ifndef NAVIT_BT_NAVIGATOR__NAVIGATE_THROUTH_PATH_ACTION_SERVER_H
#define NAVIT_BT_NAVIGATOR__NAVIGATE_THROUTH_PATH_ACTION_SERVER_H

#include <utility>
#include <fstream>

#include <navit_msgs/NavigateThroughPathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Polygon.h>

#include "navit_bt_navigator/action_server_base.hpp"
#include "navit_bt_navigator/navit_bt_runner.h"

namespace navit_bt_navigator
{
class NavigateThroughPathActionServer : public ActionServerBase<navit_msgs::NavigateThroughPathAction>
{
public:
  inline static const std::string action_name = "/navigate_through_path";

  explicit NavigateThroughPathActionServer(ros::NodeHandle& nh, std::string xml_string = "")
    : ActionServerBase(nh, action_name), default_xml_string_(std::move(xml_string))
  {
  }

protected:
  void executeCB(const GoalTypeConstPtr& goal) override;

private:
  std::string default_xml_string_;
  const std::string default_xml_string = R"(
    <root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
        <BehaviorTree ID="BehaviorTree">
            <Sequence>
                <Action ID="FollowPath" controller_id="ControllerPathFollow" path="{path}" server_name="/follow_path" server_timeout="0.5"/>
            </Sequence>
        </BehaviorTree>
    </root>
)";
};
}  // namespace navit_bt_navigator
#endif  // NAVIT_BT_NAVIGATOR__NAVIGATE_THROUTH_PATH_ACTION_SERVER_H
