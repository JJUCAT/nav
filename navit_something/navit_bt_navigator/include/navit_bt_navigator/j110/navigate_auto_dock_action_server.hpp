#ifndef NAVIT_BT_NAVIGATOR__NAVIGATE_AUTO_DOCK_ACTION_SERVER_H
#define NAVIT_BT_NAVIGATOR__NAVIGATE_AUTO_DOCK_ACTION_SERVER_H

#include <utility>
#include <fstream>

#include <navit_msgs/NavigateAutoDockAction.h>
#include <nav_msgs/Path.h>

#include "navit_bt_navigator/action_server_base.hpp"
#include "navit_bt_navigator/navit_bt_runner.h"

namespace navit_bt_navigator
{
class NavigateAutoDockActionServer : public ActionServerBase<navit_msgs::NavigateAutoDockAction>
{
public:
  inline static const std::string action_name = "/navigate_auto_dock";

  explicit NavigateAutoDockActionServer(ros::NodeHandle& nh, std::string xml_string = "")
    : ActionServerBase(nh, action_name), default_xml_string_(std::move(xml_string))
  {
  }

protected:
  void executeCB(const GoalTypeConstPtr& goal) override;

private:
  std::string default_xml_string_;
  const std::string default_xml_string = R"(
    <?xml version="1.0"?>
    <root main_tree_to_execute="BehaviorTree">
        <!-- ////////// -->
        <BehaviorTree ID="BehaviorTree">
            <Sequence>
                <Action ID="ApproachDock" controller_plugin_name="" dock_pose="{dock_pose}" expected_dock_pose="{expected_dock_pose}" filter_plugin_name="ekf_1" goal_reached="{goal_reached}" percepted_pose_offset="" perception_plugin_name="" server_name="/approach_dock" server_timeout="0.5"/>
                <Action ID="FinalDock" dock_finished="" dock_pose="{dock_pose}" rotate_in_place="" server_name="/final_dock" server_timeout="0.5"/>
            </Sequence>
        </BehaviorTree>
    </root>
)";
};
}  // namespace navit_bt_navigator
#endif  // NAVIT_BT_NAVIGATOR__AUTO_DOCK_ACTION_SERVER_H
