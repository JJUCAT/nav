#ifndef NAVIT_BT_NAVIGATOR__AUTO_DOCK_ACTION_SERVER_H
#define NAVIT_BT_NAVIGATOR__AUTO_DOCK_ACTION_SERVER_H

#include <utility>
#include <fstream>

#include <navit_msgs/AutoDockAction.h>
#include <nav_msgs/Path.h>

#include "navit_bt_navigator/action_server_base.hpp"
#include "navit_bt_navigator/navit_bt_runner.h"

namespace navit_bt_navigator
{
class AutoDockActionServer : public ActionServerBase<navit_msgs::AutoDockAction>
{
public:
  inline static const std::string action_name = "/auto_dock_task";

  explicit AutoDockActionServer(ros::NodeHandle& nh, std::string xml_string = "")
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
                <Control ID="RecoveryNode" number_of_retries="8">
                    <Sequence>
                        <Decorator ID="GetPositionInFrame" current_pos="{current_pos}" frame_id="map">
                            <Delay delay_msec="3000">
                                <Decorator ID="SelectNearestPose" current_pos="{current_pos}" reference_path="{reference_line}" step_length="1.0" target_point="{target_point}">
                                    <Decorator ID="GoalUpdater" can_be_preempted="false" goal_updated="{goal_update}" input_goal="{target_point}" is_docking="true" output_goal="{output_goal}">
                                        <Action ID="ComputePathToPose" goal="{output_goal}" path="{replan_path}" planner_id="topoint" server_name="/compute_path" server_timeout="0.5" start="false"/>
                                    </Decorator>
                                </Decorator>
                            </Delay>
                        </Decorator>
                    </Sequence>
                    <Sequence>
                        <Action ID="ClearEntireCostmap" server_timeout="0.5" service_name="/move_base/clear_local_costmaps"/>
                        <Action ID="ClearEntireCostmap" server_timeout="0.5" service_name="clear_controller_costmap"/>
                    </Sequence>
                </Control>
                <Action ID="FollowPath" controller_id="path_follower" name="返回参考线控制器" path="{replan_path}" server_name="/follow_path" server_timeout="0.2"/>
            </Sequence>
        </BehaviorTree>
    </root>
)";
};
}  // namespace navit_bt_navigator
#endif  // NAVIT_BT_NAVIGATOR__AUTO_DOCK_ACTION_SERVER_H
