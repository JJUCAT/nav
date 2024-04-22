#ifndef NAVIT_BT_NAVIGATOR__NAVIGATE_TO_POSE_ACTION_SERVER_H
#define NAVIT_BT_NAVIGATOR__NAVIGATE_TO_POSE_ACTION_SERVER_H

#include <utility>
#include <fstream>

#include <navit_msgs/NavigateToPoseAction.h>
#include <nav_msgs/Path.h>

#include "navit_bt_navigator/action_server_base.hpp"
#include "navit_bt_navigator/navit_bt_runner.h"

namespace navit_bt_navigator
{
class NavigateToPoseActionServer : public ActionServerBase<navit_msgs::NavigateToPoseAction>
{
public:
  inline static const std::string action_name = "/navigate_to_pose";

  explicit NavigateToPoseActionServer(ros::NodeHandle& nh, std::string xml_string = "")
    : ActionServerBase(nh, action_name), default_xml_string_(std::move(xml_string))
  {
  }

protected:
  void executeCB(const GoalTypeConstPtr& goal) override;

private:
  std::string default_xml_string_;
  ros::NodeHandle nh_;
  const std::string default_xml_string = R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <Fallback>
                <Action ID="ComputePathToPose" error_code="{error_code}" goal="{goal}" path="{global_path}" planner_id="SmallMapSmacHybrid" server_name="/compute_path" server_timeout="0.5" start="false"/>
                <Switch2 case_1="208" case_2="200" variable="{error_code}">
                    <RetryUntilSuccessful num_attempts="2">
                        <Sequence>
                            <Action ID="MoveBackAction" back="0.3" speed="0.1"/>
                            <Action ID="ComputePathToPose" error_code="{error_code}" goal="{goal}" path="{global_path}" planner_id="SmallMapSmacHybrid" server_name="/compute_path" server_timeout="0.5" start="false"/>
                        </Sequence>
                    </RetryUntilSuccessful>
                    <Action ID="ComputePathToPose" error_code="{error_code}" goal="{goal}" path="{global_path}" planner_id="HugeMapSmacHybrid" server_name="/compute_path" server_timeout="0.5" start="false"/>
                    <AlwaysFailure/>
                </Switch2>
            </Fallback>
            <Action ID="CloseObstacleAvoidance" server_timeout="1.0" service_name="/toggle_avoidance" use_avoidance="true"/>
            <Action ID="FollowPath" controller_id="ControllerPathFollow" path="{global_path}" server_name="/follow_path" server_timeout="0.5"/>
            <Action Controller_path="{global_path}" ID="PathToPoint" tracking_path="{task_point_ros}"/>
            <Action ID="CloseObstacleAvoidance" server_timeout="1.0" service_name="/toggle_avoidance" use_avoidance="false"/>
            <Action ID="FollowPath" controller_id="tracking" path="{task_point_ros}" server_name="/follow_path" server_timeout="0.5"/>
        </Sequence>
    </BehaviorTree>
    <!-- ////////// -->
    <TreeNodesModel>
        <Action ID="CloseObstacleAvoidance">
            <input_port name="server_timeout"/>
            <input_port name="service_name"/>
            <input_port default="false" name="use_avoidance"/>
        </Action>
        <Action ID="ComputePathToPose">
            <output_port default="{error_code}" name="error_code"/>
            <input_port default="{task_pose_ros}" name="goal"/>
            <output_port default="{global_path}" name="path"/>
            <input_port default="SmallMapSmacHybrid" name="planner_id"/>
            <input_port default="/compute_path" name="server_name"/>
            <input_port default="0.5" name="server_timeout"/>
            <input_port default="false" name="start"/>
        </Action>
        <Action ID="FollowPath">
            <input_port default="ControllerPathFollow" name="controller_id"/>
            <input_port default="{task_path_ros}" name="path"/>
            <input_port default="/follow_path" name="server_name"/>
            <input_port default="0.5" name="server_timeout"/>
        </Action>
        <Action ID="MoveBackAction">
            <input_port default="1.6" name="back"/>
            <input_port default="0.2" name="speed"/>
        </Action>
        <Action ID="PathToPoint">
            <input_port default="{task_path_ros}" name="Controller_path"/>
            <output_port default="{task_point_ros}" name="tracking_path"/>
        </Action>
    </TreeNodesModel>
    <!-- ////////// -->
</root>

  )";
};
}  // namespace navit_bt_navigator
#endif  // NAVIT_BT_NAVIGATOR__NAVIGATE_TO_POSE_ACTION_SERVER_H
